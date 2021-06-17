#!/usr/bin/bash

if [ $1 -eq 0 ]; then
  printf %s "0" > /data/params/d/dp_jetson
  cd /data/openpilot || exit
  git reset --hard @{u}
  git clean -xdf
fi
reboot
