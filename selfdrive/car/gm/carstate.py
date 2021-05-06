from cereal import car
from common.numpy_fast import mean
from selfdrive.config import Conversions as CV
from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from selfdrive.car.interfaces import CarStateBase
from selfdrive.car.gm.values import DBC, CAR, AccState, CanBus, \
                                    CruiseButtons, STEER_THRESHOLD
from selfdrive.kegman_kans_conf import kegman_kans_conf
kegman_kans = kegman_kans_conf()

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]['pt'])
    self.shifter_values = can_define.dv["ECMPRDNL"]["PRNDL"]

    self.prev_distance_button = 0
    self.prev_lka_button = 0
    self.lka_button = 0
    self.distance_button = 0
    self.follow_level = 2
    self.lkMode = True
    self.autoHold = False
    self.autoHoldActive = False
    self.autoHoldActivated = False
    self.regenPaddlePressed = 0
    self.cruiseMain = False
    self.engineRPM = 0


  def update(self, pt_cp, ch_cp): # line for brake light
    ret = car.CarState.new_message()

    self.prev_cruise_buttons = self.cruise_buttons
    self.cruise_buttons = pt_cp.vl["ASCMSteeringButton"]['ACCButtons']
# 4 lines for 3Bar Distance	
    self.prev_lka_button = self.lka_button
    self.lka_button = pt_cp.vl["ASCMSteeringButton"]["LKAButton"]
    self.prev_distance_button = self.distance_button
    self.distance_button = pt_cp.vl["ASCMSteeringButton"]["DistanceButton"]

# 6 lines for Match OP's speed to Cluster's
#    ret.wheelSpeeds.fl = pt_cp.vl["EBCMWheelSpdFront"]['FLWheelSpd'] * CV.KPH_TO_MS
#    ret.wheelSpeeds.fr = pt_cp.vl["EBCMWheelSpdFront"]['FRWheelSpd'] * CV.KPH_TO_MS
#    ret.wheelSpeeds.rl = pt_cp.vl["EBCMWheelSpdRear"]['RLWheelSpd'] * CV.KPH_TO_MS
#    ret.wheelSpeeds.rr = pt_cp.vl["EBCMWheelSpdRear"]['RRWheelSpd'] * CV.KPH_TO_MS
#    ret.vEgoRaw = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr])*(80./78.)
    ret.vEgoRaw = pt_cp.vl["ECMVehicleSpeed"]['VehicleSpeed'] * CV.MPH_TO_MS # * (80./78.)
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < 0.01

    ret.steeringAngleDeg = pt_cp.vl["PSCMSteeringAngle"]['SteeringWheelAngle']
    ret.steeringRateDeg = pt_cp.vl["PSCMSteeringAngle"]['SteeringWheelRate']
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(pt_cp.vl["ECMPRDNL"]['PRNDL'], None))
    ret.brake = pt_cp.vl["EBCMBrakePedalPosition"]['BrakePedalPosition'] / 0xd0
    # Brake pedal's potentiometer returns near-zero reading even when pedal is not pressed.
    if ret.brake < 10/0xd0:
      ret.brake = 0.

    ret.gas = pt_cp.vl["AcceleratorPedal"]['AcceleratorPedal'] / 254.
    ret.gasPressed = ret.gas > 1e-5


    ret.steeringTorque = pt_cp.vl["PSCMStatus"]['LKADriverAppldTrq']
    ret.steeringTorqueEps = pt_cp.vl["PSCMStatus"]['LKATorqueDelivered']
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD

    # 1 - open, 0 - closed
    ret.doorOpen = (pt_cp.vl["BCMDoorBeltStatus"]['FrontLeftDoor'] == 1 or
                    pt_cp.vl["BCMDoorBeltStatus"]['FrontRightDoor'] == 1 or
                    pt_cp.vl["BCMDoorBeltStatus"]['RearLeftDoor'] == 1 or
                    pt_cp.vl["BCMDoorBeltStatus"]['RearRightDoor'] == 1)

    # 1 - latched
    ret.seatbeltUnlatched = pt_cp.vl["BCMDoorBeltStatus"]['LeftSeatBelt'] == 0
    ret.leftBlinker = pt_cp.vl["BCMTurnSignals"]['TurnSignals'] == 1
    ret.rightBlinker = pt_cp.vl["BCMTurnSignals"]['TurnSignals'] == 2

    self.park_brake = pt_cp.vl["EPBStatus"]['EPBClosed']
    ret.cruiseState.available = bool(pt_cp.vl["ECMEngineStatus"]['CruiseMainOn'])
    self.cruiseMain = ret.cruiseState.available
    ret.espDisabled = pt_cp.vl["ESPStatus"]['TractionControlOn'] != 1
    self.pcm_acc_status = pt_cp.vl["AcceleratorPedal2"]['CruiseState']

    ret.brakePressed = ret.brake > 1e-5
    # Regen braking is braking
    if self.car_fingerprint == CAR.VOLT:
      self.regenPaddlePressed = bool(pt_cp.vl["EBCMRegenPaddle"]['RegenPaddle'])
      ret.brakePressed = ret.brakePressed or self.regenPaddlePressed

    ret.cruiseState.enabled = self.pcm_acc_status != AccState.OFF
    ret.cruiseState.standstill = False

    # 0 - inactive, 1 - active, 2 - temporary limited, 3 - failed
    self.lkas_status = pt_cp.vl["PSCMStatus"]['LKATorqueDeliveredStatus']
    ret.steerWarning = self.lkas_status not in [0, 1]

    self.engineRPM = pt_cp.vl["ECMEngineStatus"]['EngineRPM']

# bellow line for Brake Light	
    ret.brakeLights = ch_cp.vl["EBCMFrictionBrakeStatus"]["FrictionBrakePressure"] != 0 or ret.brakePressed

# belows are for GM's Autihold
    if kegman_kans.conf['AutoHold'] == "1":
      self.autoHold = True
    else:
      self.autoHold = False

    ret.autoHoldActivated = self.autoHoldActivated
    return ret
# 2 lines for 3 Bar distance
  def get_follow_level(self):
    return self.follow_level  
  
  @staticmethod
  def get_can_parser(CP):
    # this function generates lists for signal, messages and initial values
    signals = [
      # sig_name, sig_address, default
      ("BrakePedalPosition", "EBCMBrakePedalPosition", 0),
      ("FrontLeftDoor", "BCMDoorBeltStatus", 0),
      ("FrontRightDoor", "BCMDoorBeltStatus", 0),
      ("RearLeftDoor", "BCMDoorBeltStatus", 0),
      ("RearRightDoor", "BCMDoorBeltStatus", 0),
      ("LeftSeatBelt", "BCMDoorBeltStatus", 0),
      ("RightSeatBelt", "BCMDoorBeltStatus", 0),
      ("TurnSignals", "BCMTurnSignals", 0),
      ("AcceleratorPedal", "AcceleratorPedal", 0),
      ("CruiseState", "AcceleratorPedal2", 0),
      ("ACCButtons", "ASCMSteeringButton", CruiseButtons.UNPRESS),
      ("SteeringWheelAngle", "PSCMSteeringAngle", 0),
      ("SteeringWheelRate", "PSCMSteeringAngle", 0),
      ("FLWheelSpd", "EBCMWheelSpdFront", 0),
      ("FRWheelSpd", "EBCMWheelSpdFront", 0),
      ("RLWheelSpd", "EBCMWheelSpdRear", 0),
      ("RRWheelSpd", "EBCMWheelSpdRear", 0),
      ("PRNDL", "ECMPRDNL", 0),
      ("LKADriverAppldTrq", "PSCMStatus", 0),
      ("LKATorqueDelivered", "PSCMStatus", 0),
      ("LKATorqueDeliveredStatus", "PSCMStatus", 0),
      ("TractionControlOn", "ESPStatus", 0),
      ("EPBClosed", "EPBStatus", 0),
      ("CruiseMainOn", "ECMEngineStatus", 0),
      ("LKAButton", "ASCMSteeringButton", 0),
      ("DistanceButton", "ASCMSteeringButton", 0),
      ("LKATorqueDelivered", "PSCMStatus", 0),
      ("EngineRPM", "ECMEngineStatus", 0),
      ("VehicleSpeed", "ECMVehicleSpeed", 0),
    ]

    if CP.carFingerprint == CAR.VOLT:
      signals += [
        ("RegenPaddle", "EBCMRegenPaddle", 0),
      ]

    return CANParser(DBC[CP.carFingerprint]['pt'], signals, [], CanBus.POWERTRAIN)

# all bellows are for Brake Light
  @staticmethod
  def get_chassis_can_parser(CP):
    # this function generates lists for signal, messages and initial values
    signals = [
      # sig_name, sig_address, default
      ("FrictionBrakePressure", "EBCMFrictionBrakeStatus", 0),
    ]

    return CANParser(DBC[CP.carFingerprint]['chassis'], signals, [], CanBus.CHASSIS)