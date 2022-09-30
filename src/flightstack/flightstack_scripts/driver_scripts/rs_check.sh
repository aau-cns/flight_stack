#!/bin/sh
# Copyright (C) 2021 Martin Scheiber, Control of Networked Systems, University of Klagenfurt, Austria.
#
# All rights reserved.
#
# This software is licensed under the terms of the BSD-2-Clause-License with
# no commercial use allowed, the full terms of which are made available
# in the LICENSE file. No license in patents is granted.
#
# You can contact the author at <martin.scheiber@ieee.org>

# this bash script adheres to linux exit codes
# see https://tldp.org/LDP/abs/html/exitcodes.html for further information

# echo "[BASH - RS] checking sensor - Realsense"

RS_CHECK="$(rs-enumerate-devices)"
# echo "[BASH - RS] status: ${RS_CHECK}"

# check if device is found in lsusb
RS_ID="8087:0b37"
RS_STATUS="$(ssh core@10.42.0.102 'lsusb' | grep ${RS_ID})"
# echo "[BASH - RS] status: ${RS_STATUS}"

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
