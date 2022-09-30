#!/bin/bash
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

#echo "[BASH - IDS] checking sensor - IDS Camera"

IDS_STATUS="$(systemctl status ueyeusbdrc)"
#echo "[BASH - IDS] status: ${IDS_STATUS}"

# give it a couple more secs if driver has just restarted
IDS_RESTART_CHECK=$(echo "$IDS_STATUS" | grep "Model")
if [[ -z "${IDS_RESTART_CHECK}" ]]; then
  sleep 2
  IDS_STATUS="$(systemctl status ueyeusbdrc)"
fi

case $IDS_STATUS in
  "")
    # service cannot be found
    echo "[BASH - IDS] Service cannot be found"
    exit 1
    ;;
  *)
    # check if service is active
    IS_ACTIVE=$(echo "$IDS_STATUS" | grep "Active: active (running)")
    if [[ -z "${IS_ACTIVE}" ]]; then
      # service is not active
      echo "[BASH - IDS] service is not active"
      exit 1
    else
      # check if status contains fail pattern
      HAS_FAILED=$(echo "$IDS_STATUS" | grep "0xffffffff")
      if [[ -z "${HAS_FAILED}" ]]; then
        # when empty driver is ok
        #echo "[BASH - IDS] camera driver is OK"
        exit 0
      else
        # driver loading has failed
        echo "[BASH - IDS] driver start has failed"
        exit 1
      fi
    fi
    ;;
esac

# in case something went wrong
exit 1
