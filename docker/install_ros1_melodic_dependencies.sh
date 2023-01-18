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
  python-catkin-tools \
  python-rosdep \
  ros-melodic-dynamic-reconfigure \
  ros-melodic-eigen-conversions \
  ros-melodic-tf \
  ros-melodic-tf2-ros \
  ros-melodic-tf2-eigen \
  ros-melodic-nodelet \
  ros-melodic-geographic-msgs \
  ros-melodic-angles \
  ros-melodic-diagnostic-updater \
  ros-melodic-urdf \
  ros-melodic-control-toolbox \
  python-rosinstall-generator \
  python-future \
  python-lxml

# install geopgracilib dataset
mkdir -p /opt/flightstack && cd /opt/flightstack
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod +x install_geographiclib_datasets.sh
./install_geographiclib_datasets.sh

set +x
