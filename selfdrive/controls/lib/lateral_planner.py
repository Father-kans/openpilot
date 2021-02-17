import os
import math
import numpy as np
from common.realtime import sec_since_boot, DT_MDL
from selfdrive.car.gm.values import CAR
from common.numpy_fast import interp
from selfdrive.swaglog import cloudlog
from selfdrive.controls.lib.lateral_mpc import libmpc_py
from selfdrive.controls.lib.drive_helpers import MPC_COST_LAT, MPC_N, CAR_ROTATION_RADIUS
from selfdrive.controls.lib.lane_planner import LanePlanner, TRAJECTORY_SIZE
from selfdrive.config import Conversions as CV
from common.params import Params
from selfdrive.kegman_kans_conf import kegman_kans_conf
import cereal.messaging as messaging
from cereal import log
from selfdrive.ntune import ntune_get, ntune_isEnabled

LaneChangeState = log.LateralPlan.LaneChangeState
LaneChangeDirection = log.LateralPlan.LaneChangeDirection

LOG_MPC = os.environ.get('LOG_MPC', False)

LANE_CHANGE_SPEED_MIN = 15 * CV.MPH_TO_MS
LANE_CHANGE_TIME_MAX = 10.

DESIRES = {
  LaneChangeDirection.none: {
    LaneChangeState.off: log.LateralPlan.Desire.none,
    LaneChangeState.preLaneChange: log.LateralPlan.Desire.none,
    LaneChangeState.laneChangeStarting: log.LateralPlan.Desire.none,
    LaneChangeState.laneChangeFinishing: log.LateralPlan.Desire.none,
  },
  LaneChangeDirection.left: {
    LaneChangeState.off: log.LateralPlan.Desire.none,
    LaneChangeState.preLaneChange: log.LateralPlan.Desire.none,
    LaneChangeState.laneChangeStarting: log.LateralPlan.Desire.laneChangeLeft,
    LaneChangeState.laneChangeFinishing: log.LateralPlan.Desire.laneChangeLeft,
  },
  LaneChangeDirection.right: {
    LaneChangeState.off: log.LateralPlan.Desire.none,
    LaneChangeState.preLaneChange: log.LateralPlan.Desire.none,
    LaneChangeState.laneChangeStarting: log.LateralPlan.Desire.laneChangeRight,
    LaneChangeState.laneChangeFinishing: log.LateralPlan.Desire.laneChangeRight,
  },
}


def calc_states_after_delay(states, v_ego, steer_angle, curvature_factor, steer_ratio, delay):
  states[0].x = v_ego * delay
  states[0].psi = v_ego * curvature_factor * math.radians(steer_angle) / steer_ratio * delay
  states[0].y = states[0].x * math.sin(states[0].psi / 2)
  return states

class LateralPlanner():
  def __init__(self, CP):
    self.LP = LanePlanner()

    self.last_cloudlog_t = 0
    #self.steer_rate_cost = CP.steerRateCost
    self.steer_rate_cost_prev = ntune_get('steerRateCost')
    self.steer_actuator_delay_prev = ntune_get('steerActuatorDelay')

    self.setup_mpc()
    self.solution_invalid_cnt = 0
    self.lane_change_enabled = Params().get('LaneChangeEnabled') == b'1'
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.lane_change_ll_prob = 1.0
    self.prev_one_blinker = False
    self.desire = log.LateralPlan.Desire.none

    self.path_xyz = np.zeros((TRAJECTORY_SIZE,3))
    self.plan_yaw = np.zeros((TRAJECTORY_SIZE,))
    self.t_idxs = np.arange(TRAJECTORY_SIZE)
    self.y_pts = np.zeros(TRAJECTORY_SIZE)

    self.mpc_frame = 0
    self.sR_time = 1
    self.sR_delay_counter = 0
    self.v_ego_ed = 0.0

    self.use_dynamic_sr = CP.carName in [CAR.VOLT]

    kegman_kans = kegman_kans_conf(CP)
    self.alc_nudge_less = bool(int(kegman_kans.conf['ALCnudgeLess']))
    self.alc_min_speed = float(kegman_kans.conf['ALCminSpeed'])
    self.alc_timer = float(kegman_kans.conf['ALCtimer'])

  def setup_mpc(self):
    self.libmpc = libmpc_py.libmpc
    self.libmpc.init(MPC_COST_LAT.PATH, MPC_COST_LAT.HEADING, self.steer_rate_cost_prev)

    self.mpc_solution = libmpc_py.ffi.new("log_t *")
    self.cur_state = libmpc_py.ffi.new("state_t *")
    self.cur_state[0].x = 0.0
    self.cur_state[0].y = 0.0
    self.cur_state[0].psi = 0.0
    self.cur_state[0].curvature = 0.0

    self.angle_steers_des = 0.0
    self.angle_steers_des_mpc = 0.0
    self.angle_steers_des_prev = 0.0
    self.angle_steers_des_time = 0.0

  def update(self, sm, CP, VM):

    v_ego = sm['carState'].vEgo
    active = sm['controlsState'].active
    # angle_offset = sm['liveParameters'].angleOffset
    steering_wheel_angle_offset_deg = sm['liveParameters'].angleOffset
    # angle_steers = sm['carState'].steeringAngle
    steering_wheel_angle_deg = sm['carState'].steeringAngle

    if self.steer_rate_cost_prev != ntune_get('steerRateCost'):
      self.steer_rate_cost_prev = ntune_get('steerRateCost')

    if self.steer_actuator_delay_prev != ntune_get('steerActuatorDelay'):
      self.steer_actuator_delay_prev = ntune_get('steerActuatorDelay')

      self.libmpc.init(MPC_COST_LAT.PATH, MPC_COST_LAT.HEADING, self.steer_rate_cost_prev)
      self.cur_state[0].curvature = math.radians(steering_wheel_angle_deg - steering_wheel_angle_offset_deg) / VM.sR

    # Run MPC
    self.angle_steers_des_prev = self.angle_steers_des_mpc

    # Update vehicle model
    x = max(sm['liveParameters'].stiffnessFactor, 0.1)

    if self.use_dynamic_sr:
      sr = interp(abs(self.angle_steers_des_mpc), [5., 35.], [13.5, 17.5])
    else:
      if ntune_isEnabled('useLiveSteerRatio'):
        sr = max(sm['liveParameters'].steerRatio, 0.1)
      else:
        sr = max(ntune_get('steerRatio'), 0.1)

    VM.update_params(x, sr)

    curvature_factor = VM.curvature_factor(v_ego)

    # Get sR, BP, Time from kegman.json every x seconds
    self.mpc_frame += 1
    if self.mpc_frame % 500 == 0:
      kegman_kans = kegman_kans_conf()
      if kegman_kans.conf['nTune'] == "1":
        self.sR = [ntune_get('steerRatio'), ntune_get('steerRatio') + float(kegman_kans.conf['sR_boost'])]
        self.sRBP = [float(kegman_kans.conf['sR_BP0']), float(kegman_kans.conf['sR_BP1'])]
        self.sR_time = int(float(kegman_kans.conf['sR_time'])) * 100

      self.mpc_frame = 0

    measured_curvature = -curvature_factor * math.radians(steering_wheel_angle_deg - steering_wheel_angle_offset_deg) / VM.sR


    md = sm['modelV2']
    self.LP.parse_model(sm['modelV2'])
    if len(md.position.x) == TRAJECTORY_SIZE and len(md.orientation.x) == TRAJECTORY_SIZE:
      self.path_xyz = np.column_stack([md.position.x, md.position.y, md.position.z])
      self.t_idxs = np.array(md.position.t)
      self.plan_yaw = list(md.orientation.z)

    # Lane change logic
    lane_change_direction = LaneChangeDirection.none
    one_blinker = sm['carState'].leftBlinker != sm['carState'].rightBlinker
    below_lane_change_speed = v_ego < self.alc_min_speed

    if not active or self.lane_change_timer > 10.0:
      self.lane_change_state = LaneChangeState.off
      self.pre_lane_change_timer = 0.0
    else:
      if sm['carState'].leftBlinker:
        self.lane_change_direction = LaneChangeDirection.left
        self.pre_lane_change_timer += DT_MDL
      elif sm['carState'].rightBlinker:
        self.lane_change_direction = LaneChangeDirection.right
        self.pre_lane_change_timer += DT_MDL
      else:
        self.pre_lane_change_timer = 0.0

      if self.alc_nudge_less and self.pre_lane_change_timer > self.alc_timer:
        torque_applied = True

      else:
        if lane_change_direction == LaneChangeDirection.left:
          torque_applied = sm['carState'].steeringTorque > 0 and sm['carState'].steeringPressed
        else:
          torque_applied = sm['carState'].steeringTorque < 0 and sm['carState'].steeringPressed


      blindspot_detected = ((sm['carState'].leftBlindspot and self.lane_change_direction == LaneChangeDirection.left) or
                            (sm['carState'].rightBlindspot and self.lane_change_direction == LaneChangeDirection.right))

      lane_change_prob = self.LP.l_lane_change_prob + self.LP.r_lane_change_prob

      # State transitions
      # off
      if self.lane_change_state == LaneChangeState.off and one_blinker and not self.prev_one_blinker and not below_lane_change_speed:
        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_ll_prob = 1.0

      # pre
      elif self.lane_change_state == LaneChangeState.preLaneChange:
        if not one_blinker or below_lane_change_speed:
          self.lane_change_state = LaneChangeState.off
        elif torque_applied and not blindspot_detected:
          self.lane_change_state = LaneChangeState.laneChangeStarting

      # starting
      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        # fade out over .5s
        self.lane_change_ll_prob = max(self.lane_change_ll_prob - 2*DT_MDL, 0.0)
        # 98% certainty
        if lane_change_prob < 0.02 and self.lane_change_ll_prob < 0.01:
          self.lane_change_state = LaneChangeState.laneChangeFinishing

      # finishing
      elif self.lane_change_state == LaneChangeState.laneChangeFinishing:
        # fade in laneline over 1s
        self.lane_change_ll_prob = min(self.lane_change_ll_prob + DT_MDL, 1.0)
        if one_blinker and self.lane_change_ll_prob > 0.99:
          self.lane_change_state = LaneChangeState.preLaneChange
        elif self.lane_change_ll_prob > 0.99:
          self.lane_change_state = LaneChangeState.off

    if self.lane_change_state in [LaneChangeState.off, LaneChangeState.preLaneChange]:
      self.lane_change_timer = 0.0
    else:
      self.lane_change_timer += DT_MDL

    self.prev_one_blinker = one_blinker

    self.desire = DESIRES[self.lane_change_direction][self.lane_change_state]

    # Turn off lanes during lane change
    if self.desire == log.LateralPlan.Desire.laneChangeRight or self.desire == log.LateralPlan.Desire.laneChangeLeft:
      self.LP.lll_prob *= self.lane_change_ll_prob
      self.LP.rll_prob *= self.lane_change_ll_prob
    d_path_xyz = self.LP.get_d_path(v_ego, self.t_idxs, self.path_xyz)
    y_pts = np.interp(v_ego * self.t_idxs[:MPC_N+1], np.linalg.norm(d_path_xyz, axis=1), d_path_xyz[:,1])
    heading_pts = np.interp(v_ego * self.t_idxs[:MPC_N+1], np.linalg.norm(self.path_xyz, axis=1), self.plan_yaw)
    self.y_pts = y_pts

    steerActuatorDelay = ntune_get('steerActuatorDelay')

    # account for actuation delay
    self.cur_state = calc_states_after_delay(self.cur_state, v_ego, steering_wheel_angle_deg - steering_wheel_angle_offset_deg, curvature_factor, VM.sR,
                                             steerActuatorDelay)

    assert len(y_pts) == MPC_N + 1
    assert len(heading_pts) == MPC_N + 1
    self.libmpc.run_mpc(self.cur_state, self.mpc_solution,
                        float(v_ego),
                        CAR_ROTATION_RADIUS,
                        list(y_pts),
                        list(heading_pts))
    # init state for next
    self.cur_state.x = 0.0
    self.cur_state.y = 0.0
    self.cur_state.psi = 0.0
    self.cur_state.curvature = interp(DT_MDL, self.t_idxs[:MPC_N+1], self.mpc_solution.curvature)

    # TODO this needs more thought, use .2s extra for now to estimate other delays
    delay = steerActuatorDelay + .2
    next_curvature = interp(delay, self.t_idxs[:MPC_N+1], self.mpc_solution.curvature)
    psi = interp(delay, self.t_idxs[:MPC_N+1], self.mpc_solution.psi)
    next_curvature_rate = self.mpc_solution.curvature_rate[0]
    next_curvature_from_psi = psi/(max(v_ego, 1e-1) * delay)
    if psi > self.mpc_solution.curvature[0] * delay * v_ego:
      next_curvature = max(next_curvature_from_psi, next_curvature)
    else:
      next_curvature = min(next_curvature_from_psi, next_curvature)

    # reset to current steer angle if not active or overriding
    if active:
      curvature_desired = next_curvature
      desired_curvature_rate = next_curvature_rate
    else:
      curvature_desired = measured_curvature
      desired_curvature_rate = 0.0

    # negative sign, controls uses different convention
    self.desired_steering_wheel_angle_deg = -float(math.degrees(curvature_desired * VM.sR)/curvature_factor) + steering_wheel_angle_offset_deg
    self.desired_steering_wheel_angle_rate_deg = -float(math.degrees(desired_curvature_rate * VM.sR)/curvature_factor)

    #  Check for infeasable MPC solution
    mpc_nans = any(math.isnan(x) for x in self.mpc_solution.curvature)
    t = sec_since_boot()
    if mpc_nans:
      self.libmpc.init(MPC_COST_LAT.PATH, MPC_COST_LAT.HEADING, self.steer_rate_cost_prev)
      self.cur_state.curvature = measured_curvature

      if t > self.last_cloudlog_t + 5.0:
        self.last_cloudlog_t = t
        cloudlog.warning("Lateral mpc - nan: True")

    if self.mpc_solution[0].cost > 20000. or mpc_nans:   # TODO: find a better way to detect when MPC did not converge
      self.solution_invalid_cnt += 1
    else:
      self.solution_invalid_cnt = 0

  def publish(self, sm, pm, CP, VM):
    plan_solution_valid = self.solution_invalid_cnt < 2
    plan_send = messaging.new_message('lateralPlan')
    plan_send.valid = sm.all_alive_and_valid(service_list=['carState', 'controlsState', 'liveParameters', 'modelV2'])
    plan_send.lateralPlan.laneWidth = float(self.LP.lane_width)
    plan_send.lateralPlan.dPathPoints = [float(x) for x in self.y_pts]
    plan_send.lateralPlan.lProb = float(self.LP.lll_prob)
    plan_send.lateralPlan.rProb = float(self.LP.rll_prob)
    plan_send.lateralPlan.dProb = float(self.LP.d_prob)

    plan_send.lateralPlan.angleSteers = float(self.desired_steering_wheel_angle_deg)
    plan_send.lateralPlan.rateSteers = float(self.desired_steering_wheel_angle_rate_deg)
    plan_send.lateralPlan.angleOffset = float(sm['liveParameters'].angleOffset)
    plan_send.lateralPlan.mpcSolutionValid = bool(plan_solution_valid)

    plan_send.lateralPlan.desire = self.desire
    plan_send.lateralPlan.laneChangeState = self.lane_change_state
    plan_send.lateralPlan.laneChangeDirection = self.lane_change_direction

    plan_send.lateralPlan.steerRatio = VM.sR
    plan_send.lateralPlan.steerRateCost = self.steer_rate_cost_prev
    plan_send.lateralPlan.steerActuatorDelay = self.steer_actuator_delay_prev

    pm.send('lateralPlan', plan_send)

    if LOG_MPC:
      dat = messaging.new_message('liveMpc')
      dat.liveMpc.x = list(self.mpc_solution[0].x)
      dat.liveMpc.y = list(self.mpc_solution[0].y)
      dat.liveMpc.psi = list(self.mpc_solution[0].psi)
      dat.liveMpc.tire_angle = list(self.mpc_solution[0].tire_angle)
      dat.liveMpc.cost = self.mpc_solution[0].cost
      pm.send('liveMpc', dat)