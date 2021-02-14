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

bag_name="mocap_full_setup"

px4_topics=(
"/mavros/imu/data_raw"
"/mavros/imu/mag"
"/mavros/global_position/raw/fix"
"/mavros/global_position/raw/gps_vel"
"/mavros/global_position/raw/satellites"
"/mavros/imu/static_pressure"
"/mavros/imu/temperature_imu"
)

mocap_vehicle_topics=(
"/twins_three/vrpn_client/raw_transform"
)

mocap_tags_topics=(
"/tag_board_8/vrpn_client/raw_transform"
)

ids_camera_topics=(
"/mission_cam/image_raw"
"/mission_cam/camera_info"
)

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

rtk_gps1_topic=(
"/rtk_gps_1/aidalm"
"/rtk_gps_1/aideph"
"/rtk_gps_1/fix"
"/rtk_gps_1/fix_velocity"
"/rtk_gps_1/monhw"
"/rtk_gps_1/navclock"
"/rtk_gps_1/navpvt"
"/rtk_gps_1/navsat"
"/rtk_gps_1/navstatus"
"/rtk_gps_1/navsvin"
)

rtk_gps2_topic=(
"/rtk_gps_2/aidalm"
"/rtk_gps_2/aideph"
"/rtk_gps_2/fix"
"/rtk_gps_2/fix_velocity"
"/rtk_gps_2/monhw"
"/rtk_gps_2/navclock"
"/rtk_gps_2/navpvt"
"/rtk_gps_2/navsat"
"/rtk_gps_2/navstatus"
"/rtk_gps_2/navsvin"
)

lrf_topic=(
"/lidar_lite/range"
)

imu_lsm9ds1_topic=(
"/imu_lsm9ds1/imu"
"/imu_lsm9ds1/mag"
)

uwb_topic=(
"/uwb_bridge/imu"
"/uwb_bridge/uwb_meas"
)

# Generate Topic Strings Grouped by Platform Devices (concatinate string arrays)

## Module 1
### Sensors
group_mod1_sensors=(
${mocap_vehicle_topics[@]}
${px4_topics[@]}
${rtk_gps1_topic[@]}
${rtk_gps2_topic[@]}
${lrf_topic[@]}
)

topics_mod1_sensors=${group_mod1_sensors[@]}
name_dev1_sensors="_mod1_sensors"

### Camera
topics_mod1_ids_img=${ids_camera_topics[@]}
name_mod1_ids_img="_ids_img"

## Module 2
### Sensors

group_mod1_sensors=(
${real_sense_imu_odom_topics[@]}
${imu_lsm9ds1_topic[@]}
${uwb_topic[@]}
)

topics_mod2_sensors=${group_mod1_sensors[@]}
name_mod2_sensors="_mod2_sensors"

### RealSense Camera
topics_mod2_rs_img=${real_sense_cam_topics[@]}
name_mod2_rs_img="_rs_img"

# Generate Topic Strings Grouped by Topics (concatinate string arrays)

## All text/value based sensors
topics1_to_record=(
${mocap_vehicle_topics[@]}
${mocap_tags_topics[@]}
${px4_topics[@]}
${real_sense_imu_odom_topics[@]}
${rtk_gps1_topic[@]}
${rtk_gps2_topic[@]}
${lrf_topic[@]}
)

group1_to_record=${topics1_to_record[@]}
name_group1="_sensors"

## All image based sensors
### Realsense
topics2_to_record=(
${ids_camera_topics[@]}
)

group2_to_record=${topics2_to_record[@]}
name_group2="_ids"

### IDS Camera
topics3_to_record=(
${real_sense_cam_topics[@]}
)

group3_to_record=${topics3_to_record[@]}
name_group3="_realsense"

# Record options

echo "Bagname: " ${bag_name}


if [ "$1" == "dev1_full" ] ; then
	echo "Recording for device 1 (full): "
    rosbag record --tcpnodelay -b 512 --split --size=500 -o $bag_name$name_dev1_sensors ${topics_dev1_sensors} & \
    rosbag record --tcpnodelay -b 0 --split --size=1000 -o $bag_name$name_dev1_ids_img ${topics_dev1_ids_img} && kill $!

elif [ "$1" == "dev1_cam" ] ; then
	echo "Recording for device 1 (cam): "
    rosbag record --tcpnodelay -b 0 --split --size=1000 -o $bag_name$name_dev1_ids_img ${topics_dev1_ids_img}

elif [ "$1" == "dev1_sensors" ] ; then
    rosbag record --tcpnodelay -b 512 --split --size=500 -o $bag_name$name_dev1_sensors ${topics_dev1_sensors}
	echo "Recording for device 1 (sensors): "

elif [ "$1" == "dev2" ] ; then
    echo "Recording for device 2: "
    rosbag record --tcpnodelay -b 0 --split --size=1000 -o $bag_name$name_mod2_img ${topics_mod2_rs_img} & \
        rosbag record --tcpnodelay -b 0 --split --size=1000 -o $bag_name$name_mod2_sensors ${topics_mod2_sensors} && kill $!

elif [ "$1" == "dev2_cam" ] ; then
    echo "Recording for device 2 (cam): "
    rosbag record --tcpnodelay -b 0 --split --size=1000 -o $bag_name$name_mod2_rs_img ${topics_mod2_rs_img}

elif [ "$1" == "dev2_sensors" ] ; then
    echo "Recording for device 2 (sensors): "
    rosbag record --tcpnodelay -b 0 --split --size=1000 -o $bag_name$name_mod2_sensors ${topics_mod2_sensors}

elif [ "$1" == "ids" ] ; then
    echo "Group 1 topics to record: " ${group2_to_record}
    rosbag record --split --size=500 --buffsize=2048 -o $bag_name$name_group1 ${group2_to_record}
elif [ "$1" == "realsense" ] ; then
    echo "Group 3 topics to record: " ${group3_to_record}
    rosbag record --tcpnodelay -b 0 --split --size=1000 -o $bag_name$name_group3 ${group3_to_record}
elif [ "$1" == "sensors" ] ; then
    echo "Group 1 topics to record: " ${group1_to_record}
    rosbag record --split --size=500 --buffsize=2048 -o $bag_name$name_group1 ${group1_to_record}
else
    echo "Recording all defined topics!"
    echo "Group 1 topics to record: " ${group1_to_record}
    echo "Group 2 topics to record: " ${group2_to_record}
    echo "Group 3 topics to record: " ${group3_to_record}
    rosbag record --split --size=500 --buffsize=2048 -o $bag_name$name_group1 ${group1_to_record} & \
        rosbag record --split --size=500 --buffsize=2048 -o $bag_name$name_group2 ${group2_to_record} & \
        rosbag record --split --size=500 --buffsize=2048 -o $bag_name$name_group3 ${group3_to_record} && kill $!
fi;
