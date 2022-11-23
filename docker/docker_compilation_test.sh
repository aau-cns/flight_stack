#!/bin/bash -e

# Copyright (C) 2022 Martin Scheiber and Christian Brommer,
# Control of Networked Systems, University of Klagenfurt, Austria.
#
# All rights reserved.
#
# This software is licensed under the terms of the BSD-2-Clause-License with
# no commercial use allowed, the full terms of which are made available
# in the LICENSE file. No license in patents is granted.
#
# You can contact the author at <martin.scheiber@ieee.org> and
# <christian.brommer@ieee.org>.

echo "Copying Source Files..."
# cp -r /source/* /catkin_build_ws/src
# cp -r /source/* /catkin_make_ws/src

# build with catkin build
echo "Building the Appliaction with 'catkin build' ..."
cd /catkin_build_ws
catkin build -DCMAKE_BUILD_TYPE=Release -j4 -l4
catkin clean -y
catkin build -DCMAKE_BUILD_TYPE=Debug -j4 -l4
echo "Building the Appliaction with 'catkin build' - DONE"
# clean workspace to free up used disk space
catkin clean -y
rm -rf .catkin_tools

# build with catkin_make
echo "Building the Appliaction with 'catkin_make' ..."
cd /catkin_make_ws
catkin_make_isolated -DCMAKE_BUILD_TYPE=Release -j4 -l4
rm -rf build develop
catkin_make_isolated -DCMAKE_BUILD_TYPE=Debug -j4 -l4
echo "Building the Appliaction with 'catkin_make' - DONE"
# clean workspace to free up used disk space
rm -rf build develop
