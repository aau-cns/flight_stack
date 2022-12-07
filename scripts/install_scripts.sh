#!/bin/bash

# Copyright (C) 2022 Martin Scheiber,
# Control of Networked Systems, University of Klagenfurt, Austria.
#
# All rights reserved.
#
# This software is licensed under the terms of the BSD-2-Clause-License with
# no commercial use allowed, the full terms of which are made available
# in the LICENSE file. No license in patents is granted.
#
# You can contact the authors at <martin.scheiber@ieee.org>.

# install the flightstack scripts
echo "Linking flightstack scripts to /usr/bin/"
sudo ln -s $(pwd)/src/flightstack/flightstack_scripts/launch_scripts/flightstack_op.sh /usr/bin/fs_op
sudo ln -s $(pwd)/src/flightstack/flightstack_scripts/launch_scripts/flightstack_dev1.sh /usr/bin/fs_dev1
sudo ln -s $(pwd)/src/flightstack/flightstack_scripts/launch_scripts/flightstack_dev2.sh /usr/bin/fs_dev2
sudo ln -s $(pwd)/src/flightstack/flightstack_scripts/launch_scripts/flightstack_remote.sh /usr/bin/fs_remote
