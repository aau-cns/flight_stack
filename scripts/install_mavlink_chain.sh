#!/bin/bash
#description     :This script will install, setup and build the a dedicated mavlink catkin workspace.
#author		 :Christian Brommer {christian.brommer@aau.at}
#date            :20201031
#usage		 :install_mavlink_chain.sh

echo "Setup mavlink catkin workspace..."

#echo "Install related packages..."
#apt-get update && \
#apt-get install -y python-rosinstall-generator python-future python-lxml

echo "Create catkin mavros folder..."
mkdir -p ${HOME}/catkin_mavros
cd ${HOME}/catkin_mavros

echo "Init catkin workspace..."
catkin init
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
wstool init src

echo "Download mavros dependency packages to workspace..."
rosinstall_generator --rosdistro melodic mavlink | tee /tmp/mavros.rosinstall
rosinstall_generator --rosdistro melodic --upstream-development mavros --deps | tee -a /tmp/mavros.rosinstall
wstool merge -y -t src /tmp/mavros.rosinstall
wstool update -t src -j4
rosdep update --rosdistro melodic
rosdep install --rosdistro melodic -r --from-paths src --ignore-src -y
#./src/mavros/mavros/scripts/install_geographiclib_datasets.sh

echo "replacing the packages mavlink and mavros with our own repositories..."
cd ${HOME}/catkin_mavros/src
rm -rf mavros mavlink
cp -r /opt/amadee/src/mavros ./
cp -r /opt/amadee/src/mavlink ./
cd /home/${HOME}/catkin_mavros
echo "Building the catkin mavlink workspace"
catkin build
