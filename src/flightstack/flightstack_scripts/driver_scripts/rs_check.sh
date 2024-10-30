#!/bin/sh
# Copyright (C) 2023 Martin Scheiber and Christian Brommer, 
# Control of Networked Systems, University of Klagenfurt, Austria.
#
# All rights reserved.
#
# This software is licensed under the terms of the BSD-2-Clause-License with
# no commercial use allowed, the full terms of which are made available
# in the LICENSE file. No license in patents is granted.
#
# You can contact the author at <martin.scheiber@ieee.org>
# and <christian.brommer@ieee.org>.

# this bash script adheres to linux exit codes
# see https://tldp.org/LDP/abs/html/exitcodes.html for further information

RS_IS_REMOTE=false
RS_DEV_IP="10.42.0.102"
RS_DEV_USER="flightstack"


if [ ${RS_IS_REMOTE} == true ]; then
  # echo "[BASH - RS] checking sensor - Realsense"
  RS_CHECK="$(ssh ${RS_DEV_USER}@${RS_DEV_IP} 'rs-enumerate-devices')"
  # echo "[BASH - RS] status: ${RS_CHECK}"

  # check if device is found in lsusb
  RS_ID="8087:0b37"
  RS_STATUS="$(ssh ${RS_DEV_USER}@${RS_DEV_IP} 'lsusb' | grep ${RS_ID})"
  # echo "[BASH - RS] status: ${RS_STATUS}"
else
  # echo "[BASH - RS] checking sensor - Realsense"
  RS_CHECK="$(rs-enumerate-devices)"
  # echo "[BASH - RS] status: ${RS_CHECK}"

  # check if device is found in lsusb
  RS_ID="8087:0b37"
  RS_STATUS="$(lsusb | grep ${RS_ID})"
  # echo "[BASH - RS] status: ${RS_STATUS}"
fi



case $RS_STATUS in
  "")
    # device not found, return error
    echo "[BASH - RS] device cannot be found"
    exit 1
    ;;
  *)
    # device found, return success
    #echo "[BASH - RS] device found"
    exit 0
    ;;
esac

# TODO(scm): perform rs-enumerate-devices <<EOF
#decades@ubuntu:~$ rs-enumerate-devices
#No device detected. Is it plugged in?
#https://github.com/IntelRealSense/librealsense/issues/4916

# in case something went wrong
exit 1
