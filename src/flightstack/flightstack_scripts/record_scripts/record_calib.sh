#!/bin/bash

# Author: Christian Brommer

# The script records topics to individual bagfiles based on subgroups
# Subgroups can be choosen for individual recordings (e.g. different devices)
# Options:
# - ids (records ids camera images and all other non-vision sensors)
# - realsense (only records realsense images)
# - no option (records all tedinfed topics)
# the association of subgroups can be changed below

# First, list all topics specific to a sensor, then concatinate all strings for one group/device,
# then generate the final string and record ist

# Example to record all non vision sensors and the ids images:
# ./record_optitrack_full_setup ids

# Script to record the following topics:
# - MoCap (vrpn optitrack)
# - IDS camera images
# - RealSense camera images and imu
# - PX4 imu, pressure, and magnetometer

bag_name="calib"
path_local=""
path_media=""

# check for local/media paths
if [ ! -z "${2}" ]; then
  path_local="${2}/"
fi
if [ ! -z "${3}" ]; then
  path_media="${3}/"
fi

px4_topics=(
"/mavros/imu/data_raw"
"/mavros/imu/mag"
"/mavros/imu/static_pressure"
"/mavros/imu/temperature_imu"
)

bluefox_camera_topics=(
"/camera/image_raw"
"/camera/camera_info"
)

# Generate Topic Strings Grouped by Platform Devices (concatinate string arrays)

## Module 1
### Sensors
group_mod1_sensors=(
${bluefox_camera_topics[@]}
${px4_topics[@]}
)

# topics_mod1_sensors=${group_mod1_sensors[@]}
printf -v topics_mod1_sensors '%s, ' "${group_mod1_sensors[@]}"



# Record the given group of topics
echo "Bagname: ${bag_name} | ${1}"

if [ "$1" == "intrinsics" ] ; then
    echo "Recording intrinsics: "
    echo "  local path: $path_local"
    rosparam dump "${path_local}${bag_name}_params_$(date +%Y-%m-%d-%H-%M).yaml" & \
    roslaunch nodelet_rosbag nodelet_rosbag.launch start_manager:=False nodelet_manager_name:="nodelet_manager" nodelet_name:="record_cal_sensors" rosbag_path:=${path_local} rosbag_prefix:=${bag_name}_intrinsics rosbag_topics:="[${topics_mod1_sensors%,}]" && \
    kill $!

elif [ "$1" == "extrinsics" ] ; then
    echo "Recording intrinsics: "
    echo "  local path: $path_local"
    rosparam dump "${path_local}${bag_name}_params_$(date +%Y-%m-%d-%H-%M).yaml" & \
    roslaunch nodelet_rosbag nodelet_rosbag.launch start_manager:=False nodelet_manager_name:="nodelet_manager" nodelet_name:="record_cal_sensors" rosbag_path:=${path_local} rosbag_prefix:=${bag_name}_extrinsics rosbag_topics:="[${topics_mod1_sensors%,}]" && \
    kill $!

else # Handle error case
    echo -e "${RED}[ERROR] The given option is not valid! Please add the group of topics you'd like to record.${NC}"
    exit
fi;
