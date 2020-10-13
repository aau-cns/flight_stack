#!/bin/bash

# Author: Christian Brommer

# The script records topics to individual bagfiles based on subgroups
# Subgroups can be choosen for individual recordings (e.g. different devices)
# Options:
# - ids (records ids camera images and all other non-vision sensors)
# - realsense (only records realsense images)
# - no option (records all tedinfed topics)
# the association of subgroups can be changed below

# Example to record all non vision sensors and the ids images:
# ./record_optitrack_full_setup ids

# Script to record the following topics:
# - MoCap (vrpn optitrack)
# - IDS camera images
# - RealSense camera images and imu
# - PX4 imu, pressure, and magnetometer

bag_name="mocap_full_setup"

px4_topics=(
"/mavros/imu/data_raw"
"/mavros/imu/mag"
"/mavros/imu/static_pressure"
"/mavros/imu/temperature_imu"
)

mocap_topics=( "/twins_one/vrpn_client/raw_transform" )

ids_camera_topics=( "/mission_cam/image_raw" )

real_sense_imu_odom_topics=(
"/realsense/accel/imu_info"
"/realsense/accel/sample"
"/realsense/gyro/imu_info"
"/realsense/gyro/sample"
"/realsense/odom/sample"
)
real_sense_cam_topics=(
"/realsense/fisheye1/camera_info"
"/realsense/fisheye1/image_raw"
"/realsense/fisheye2/camera_info"
"/realsense/fisheye2/image_raw"
)

topics1_to_record=(
${mocap_topics[@]}
${px4_topics[@]}
${real_sense_imu_odom_topics[@]}
)

topics2_to_record=(
${ids_camera_topics[@]}
)

topics3_to_record=(
${real_sense_cam_topics[@]}
)

#Essential sensor topics
group1_to_record=${topics1_to_record[@]}
name_group1="_sensors"

#Cameras
group2_to_record=${topics2_to_record[@]}
name_group2="_ids"

group3_to_record=${topics3_to_record[@]}
name_group3="_realsense"

echo "Bagname: " ${bag_name}

if [ "$1" == "ids" ] ; then
echo "Group 1 topics to record: " ${group1_to_record}
echo "Group 2 topics to record: " ${group2_to_record}
rosbag record --split --size=500 --buffsize=2048 -o $bag_name$name_group1 ${group1_to_record} & \
rosbag record --split --size=500 --buffsize=2048 -o $bag_name$name_group2 ${group2_to_record} && kill $!
elif [ "$1" == "realsense" ] ; then
echo "Group 3 topics to record: " ${group3_to_record}
rosbag record --split --size=500 --buffsize=2048 -o $bag_name$name_group3 ${group3_to_record}
else
echo "Recording all defined topics!"
echo "Group 1 topics to record: " ${group1_to_record}
echo "Group 2 topics to record: " ${group2_to_record}
echo "Group 3 topics to record: " ${group3_to_record}
rosbag record --split --size=500 --buffsize=2048 -o $bag_name$name_group1 ${group1_to_record} & \
#rosbag record --split --size=500 --buffsize=2048 -o $bag_name$name_group2 ${group2_to_record} & \
rosbag record --split --size=500 --buffsize=2048 -o $bag_name$name_group3 ${group3_to_record} && kill $!
fi;
