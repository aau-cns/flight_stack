#!/bin/bash

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

set -x

apt update && apt install --no-install-recommends -y \
  wget \
  doxygen \
  graphviz \
  tmux \
  build-essential \
  libeigen3-dev \
  libgoogle-glog-dev \
  libgflags-dev \
  libgeographic-dev \
  geographiclib-tools \
  python3-catkin-tools \
  python3-rosdep \
  ros-noetic-dynamic-reconfigure \
  ros-noetic-eigen-conversions \
  ros-noetic-tf \
  ros-noetic-tf2-ros \
  ros-noetic-tf2-eigen \
  ros-noetic-nodelet \
  ros-noetic-geographic-msgs \
  ros-noetic-angles \
  ros-noetic-diagnostic-updater \
  ros-noetic-urdf \
  ros-noetic-control-toolbox \
  python3-rosinstall-generator \
  python3-future \
  python3-lxml

# install geopgracilib dataset
mkdir -p /opt/flightstack && cd /opt/flightstack
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod +x install_geographiclib_datasets.sh
./install_geographiclib_datasets.sh

set +x
