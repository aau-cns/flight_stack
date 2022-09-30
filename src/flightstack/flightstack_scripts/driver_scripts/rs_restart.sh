#!/bin/sh
# Copyright (C) 2021 Martin Scheiber and Christian Brommer,
# Control of Networked Systems, University of Klagenfurt, Austria.
#
# All rights reserved.
#
# This software is licensed under the terms of the BSD-2-Clause-License with
# no commercial use allowed, the full terms of which are made available
# in the LICENSE file. No license in patents is granted.
#
# You can contact the authors at <martin.scheiber@ieee.org>
# and <christian.brommer@ieee.org>

# this bash script adheres to linux exit codes
# see https://tldp.org/LDP/abs/html/exitcodes.html for further information

# restart realsense hub
# ssh core@10.42.0.102 'sudo uhubctl -a off --delay 2 -e -R -l2 -p1'
# command by chris
#ssh core@10.42.0.102 'sudo uhubctl --action cycle --location 2'
ssh core@10.42.0.102 'sudo uhubctl --action cycle --location 2 -R --delay 5'
sleep 5
ssh core@10.42.0.102 'rs-enumerate-devices'

exit 0
