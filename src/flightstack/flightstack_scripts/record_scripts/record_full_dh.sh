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

bag_name="fs_dh"
path_local=""
path_media=""

# check for local/media paths
if [ ! -z "${2}" ]; then
  path_local="${2}/"
fi
if [ ! -z "${3}" ]; then
  path_media="${3}/"
fi

RED='\033[0;31m' #Red Color
NC='\033[0m'     #No Color

px4_topics=(
"/mavros/imu/data_raw"
"/mavros/imu/mag"
"/mavros/global_position/raw/fix"
"/mavros/global_position/raw/gps_vel"
"/mavros/global_position/raw/satellites"
"/mavros/imu/static_pressure"
"/mavros/imu/temperature_imu"
"/mavros/motor_speeds/speed"
"/mavros/battery"
)

mocap_vehicle_topics=(
"/twins_five/vrpn_client/raw_transform"
)

bluefox_camera_topics=(
"/camera/image_raw"
"/camera/camera_info"
)

lrf_topic=(
"/lidar_lite/range"
)

uwb_topic=(
"/TREK1000/tagDistance_raw"
)

autonomy_topics=(
"/autonomy/request"
"/autonomy/response"
"/autonomy/logger"
"/toland/is_landed"
)

ms_topics=(
"/mission_sequencer/waypoint_list"
"/mavros/setpoint_position/local"
"/mavros/setpoint_position/global"
"/mavros/state"
"/mavros/extended_state"
)

wd_topics=(
"/watchdog/status"
"/watchdog/log"
"/watchdog/action"
)

est_topics=(
"/mavros/vision_pose/pose"
"/mavros/vision_pose/pose_cov"
"/mavros/local_position/pose"
"/mavros/local_position/pose_cov"
"/mavros/local_position/odom"
"/mavros/local_position/odom_cov"
"/uwb_init/anchors"
"/bw2_ms_msckf/pose"
)

# Generate Topic Strings Grouped by Platform Devices (concatinate string arrays)

## Module 1
### Sensors
group_mod1_sensors=(
${mocap_vehicle_topics[@]}
${px4_topics[@]}
${lrf_topic[@]}
${uwb_topic[@]}
)

# topics_mod1_sensors=${group_mod1_sensors[@]}
printf -v topics_mod1_sensors '%s, ' "${group_mod1_sensors[@]}"

### Nodes
group_mod1_nodes=(
${autonomy_topics[@]}
${ms_topics[@]}
${est_topics[@]}
${wd_topics[@]}
)

# topics_mod1_nodes=${group_mod1_nodes[@]}
printf -v topics_mod1_nodes '%s, ' "${group_mod1_nodes[@]}"

## Module 2
### Sensors

group_mod2_sensors=(
${bluefox_camera_topics[@]}
)

# topics_mod2_sensors=${group_mod2_sensors[@]}
printf -v topics_mod2_sensors '%s, ' "${group_mod2_sensors[@]}"

#
# ### RealSense Camera
# topics_mod2_rs_img=${real_sense_cam_topics[@]}
# name_mod2_rs_img="_rs_img"
#
# ### MoCap Topics
#
# group_modcap_sensors=(
# ${mocap_vehicle_topics[@]}
# ${mocap_tags_topics[@]}
# )
#
# name_mocap_sensors="_mocap"

# Generate Topic Strings Grouped by Topics (concatinate string arrays)

## All text/value based sensors
# topics1_to_record=(
# ${mocap_vehicle_topics[@]}
# ${mocap_tags_topics[@]}
# ${px4_topics[@]}
# ${real_sense_imu_odom_topics[@]}
# ${rtk_gps1_topic[@]}
# ${rtk_gps2_topic[@]}
# ${lrf_topic[@]}
# )
#
# group1_to_record=${topics1_to_record[@]}
# name_group1="_sensors"
#
# ## All image based sensors
# ### Realsense
# topics2_to_record=(
# ${ids_camera_topics[@]}
# )
#
# group2_to_record=${topics2_to_record[@]}
# name_group2="_ids"
#
# ### IDS Camera
# topics3_to_record=(
# ${real_sense_cam_topics[@]}
# )
#
# group3_to_record=${topics3_to_record[@]}
# name_group3="_realsense"


# Record the given group of topics
echo "Bagname: ${bag_name}"

if [ "$1" == "dev1_full" ] ; then
    echo "Recording for device 1 (full): "
    echo "  local path: $path_local"
    # echo "  media path: $path_media"
    # rosbag record --tcpnodelay -b 512 --split --size=500 -o "${path_local}${bag_name}_all1" ${topics_mod1_sensors} ${topics_mod1_nodes} && kill $!
    # echo "roslaunch nodelet_rosbag nodelet_rosbag.launch rosbag_path:=${path_local} rosbag_prefix:=${bag_name}_all1 rosbag_topics:=[${topics_mod1_sensors%,} ${topics_mod1_nodes%,}]" # && kill $!
    roslaunch nodelet_rosbag nodelet_rosbag.launch start_manager:=True nodelet_manager_name:="record_od1_manager" nodelet_name:="record_od1" rosbag_path:=${path_local} rosbag_prefix:=${bag_name}_all1 rosbag_topics:="[${topics_mod1_sensors%,} ${topics_mod1_nodes%,}]" && kill $!

elif [ "$1" == "dev1_cam" ] ; then
    echo "Recording for device 1 (cam): "
    rosbag record --tcpnodelay -b 0 --split --size=1000 -o $bag_name$name_mod1_ids_img ${topics_mod1_ids_img}

elif [ "$1" == "dev1_sensors" ] ; then
    rosbag record --tcpnodelay -b 512 --split --size=500 -o $bag_name$name_mod1_sensors ${topics_mod1_sensors}
	echo "Recording for device 1 (sensors): "

elif [ "$1" == "dev2_full" ] ; then
    echo "Recording for device 2 (full): "
    echo "  local path: $path_local"
    # echo "  media path: $path_media"
    # rosbag record --tcpnodelay -b 512 --split --size=500 -o "${path_local}${bag_name}_all2" ${topics_mod2_sensors} && kill $!
    # rosbag record --tcpnodelay -b 0 --split --size=1000 -o $path_media$bag_name$name_mod2_rs_img ${topics_mod2_rs_img} & \
    # rosbag record --tcpnodelay -b 0 --split --size=1000 -o $path_local$bag_name$name_mod2_sensors ${topics_mod2_sensors} && kill $!
    # roslaunch nodelet_rosbag nodelet_rosbag.launch rosbag_path:=${path_local} rosbag_prefix:="${bag_name}_all2" rosbag_topics:="[/camera/image_raw, /camera/camera_info]" && kill $!
    roslaunch nodelet_rosbag nodelet_rosbag.launch start_manager:=False nodelet_manager_name:="nodelet_manager" nodelet_name:="record_od2" rosbag_path:=${path_media} rosbag_prefix:=${bag_name}_all2 rosbag_topics:="[${topics_mod2_sensors%,}]" && kill $!

elif [ "$1" == "dev2_cam" ] ; then
    echo "Recording for device 2 (cam): "
    rosbag record --tcpnodelay -b 0 --split --size=1000 -o $bag_name$name_mod2_rs_img ${topics_mod2_rs_img}

elif [ "$1" == "dev2_sensors" ] ; then
    echo "Recording for device 2 (sensors): "
    rosbag record --tcpnodelay -b 0 --split --size=1000 -o $bag_name$name_mod2_sensors ${topics_mod2_sensors}

elif [ "$1" == "mocap" ] ; then
    echo "Recording MoCap Data: "
    rosbag record --tcpnodelay -b 512 --split --size=1000 -o $bag_name$name_mocap_sensors ${group_modcap_sensors[@]}

elif [ "$1" == "ids" ] ; then
    echo "Group 1 topics to record: " ${group2_to_record}
    rosbag record --split --size=500 --buffsize=2048 -o $bag_name$name_group1 ${group2_to_record}

elif [ "$1" == "realsense" ] ; then
    echo "Group 3 topics to record: " ${group3_to_record}
    rosbag record --tcpnodelay -b 0 --split --size=1000 -o $bag_name$name_group3 ${group3_to_record}

elif [ "$1" == "sensors" ] ; then
    echo "Group 1 topics to record: " ${group1_to_record}
    rosbag record --split --size=500 --buffsize=2048 -o $bag_name$name_group1 ${group1_to_record}

else # Handle error case
    echo -e "${RED}[ERROR] The given option is not valid! Please add the group of topics you'd like to record.${NC}"
    exit
fi;
