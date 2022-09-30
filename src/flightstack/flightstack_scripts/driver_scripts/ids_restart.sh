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

echo "[BASH] restarting service"

# check if service exists
IDS_STATUS="$(systemctl status ueyeusbdrc)"
case $IDS_STATUS in
  "")
    # service cannot be found
    echo "[BASH - IDS] Service cannot be found"
    exit 1
    ;;
  *)
    # service exists, restart it
    sudo systemctl restart ueyeusbdrc
    ;;
esac

exit 0
