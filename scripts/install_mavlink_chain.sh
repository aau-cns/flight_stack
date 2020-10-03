#!/bin/bash
apt-get update
apt-get install -y python-rosinstall-generator python-future python-lxml
apt-get install -y ros-melodic-geometry2
#assuming the catkin workspace is setup
mkdir -p /home/core/catkin_mavros/src
cd /home/core/catkin_mavros
catkin init
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

#catkin init
wstool init src
rosinstall_generator --rosdistro melodic mavlink | tee /tmp/mavros.rosinstall
rosinstall_generator --upstream-development mavros --deps | tee -a /tmp/mavros.rosinstall
wstool merge -y -t src /tmp/mavros.rosinstall
wstool update -t src -j4
rosdep update
rosdep install -r --from-paths src --ignore-src -y
./src/mavros/mavros/scripts/install_geographiclib_datasets.sh

#replacing packages with our own repositorys (mavlink, mavros)
cd /home/core/catkin_mavros/src
rm -rf mavros mavlink
cp -r /opt/amadee/src/mavros ./
cp -r /opt/amadee/src/mavlink ./

cd /home/core/catkin_ws
catkin build
