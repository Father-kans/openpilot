#!/usr/bin/bash

if [ $1 -eq 1 ]; then
  printf %s "1" > /data/params/d/CommunityFeaturesToggle

fi
if [ $1 -eq 0 ]; then
  cd /data/openpilot || exit
  git reset --hard @{u}
  git clean -xdf
fi
reboot
