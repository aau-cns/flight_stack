#!/bin/bash

# Copyright (C) 2023, Christian Brommer, Martin Scheiber,
# and others, Control of Networked Systems, University of Klagenfurt, Austria.
#
# All rights reserved.
#
# This software is licensed under the terms of the BSD-2-Clause-License with
# no commercial use allowed, the full terms of which are made available
# in the LICENSE file. No license in patents is granted.
#
# You can contact the authors at <christian.brommer@ieee.org>
# and <martin.scheiber@ieee.org>.

# The script records topics to individual bagfiles based on subgroups
# Subgroups can be choosen for individual recordings (e.g. different devices)
# Options:
# - ids (records ids camera images and all other non-vision sensors)
# - realsense (only records realsense images)
# - no option (records all tedinfed topics)
# the association of subgroups can be changed below

# First, list all topics specific to a sensor, then concatinate all strings for one group/device,
# then generate the final string and record ist

# Script to record the following topics:
# - MoCap (vrpn optitrack)
# - IDS camera images
# - RealSense camera images and imu
# - PX4 imu, pressure, and magnetometer

################################################################################
# Global Variables                                                             #
################################################################################

# setup colors
COL_ERR='\033[0;31m'  #Red Color
COL_WARN='\033[0;33m' #Yellow Color
NC='\033[0m'          #No Color

TOPIC_TYPE=full_dev1

# SCRIPT VARIABLES
bag_name="fs"
path_local=""
path_media=""

CAM_NODELET_MANAGER=record_dev2_manager
B_DEV_2_START_MANAGER=true

################################################################################
# Help                                                                         #
################################################################################
print_help(){
    echo "USAGE: ${script_name} <TOPICS> [OPTIONS]"
    echo ""
    echo "  Topics:         details the topics to record, choose one from below"
    echo "    full_dev1     all rostopics typically on dev1"
    echo "    full_dev2     all rostopics typically on dev2"
    echo ""
    echo "  Options:"
    echo "    -l PATH       path to internal media"
    echo "    -m PATH       path to external media device (for images)"
    echo "    -p NAME       prefix of bagfile, default 'fs'"
    echo ""
    echo "    -c            use mv_camera_nodelet for standard recording settings"
    echo ""
    echo "    -h        print this help"
    echo ""
    exit 0;
}

################################################################################
# Execution Options                                                            #
################################################################################

# parse TOPICS
if [ -z ${1} ]; then
  echo -e "${COL_ERR}[ERROR] TOPICS missing${NC}"
  print_help;
  exit 1;
elif [[ ! -n "${1%%-*}" ]]; then
  echo -e "${COL_ERR}[ERROR] '${1}' is an option - not a valid string${NC}"
  print_help;
  exit 1;
else
  TOPICS=${1}
fi
# shift optind to next value
OPTIND=$((OPTIND+1));


# parse flags
while getopts hcl:m:p: flag
do
    case "${flag}" in
        l) path_local=${OPTARG};;
        m) path_media=${OPTARG};;
        p) bag_name=${OPTARG};;

        c) CAM_NODELET_MANAGER=nodelet_manager;
           B_DEV_2_START_MANAGER=false;;

        h) print_help;;

        *) echo "Unknown option ${flag}"; print_help;;
    esac
done
shift $((OPTIND-1))


# check for local/media paths
if [ -z "${path_local}" ]; then
  echo -e "${COL_WARN}No local path provided, recording to home directory: '${HOME}/recordings'${NC}"
  path_local="${HOME}/recordings"
fi
if [ -z "${path_media}" ]; then
  echo -e "${COL_WARN}No media path provided, recording to local path: '${path_local}'.${NC}"
  path_media="${path_local}"
fi

# check if directories exist
if [ ! -d "${path_local}" ]; then
  echo -e "${COL_WARN}${path_local} does not exist, creating it... ${NC}"
  mkdir -p ${path_local}
fi
if [ ! -d "${path_media}" ]; then
  echo -e "${COL_WARN}${path_media} does not exist, creating it... ${NC}"
  mkdir -p ${path_media}
fi


# -----------------------------------------------------------------------------
# SETUP TOPIC GROUPS

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
)

mars_pose_topics=(
"/mars_pose_node/full_state_lite_out"
"/mars_pose_node/full_state_out"
"/mars_pose_node/odom_state_out"
"/mars_pose_node/path_state_out"
"/mars_pose_node/parameter_descriptions"
"/mars_pose_node/parameter_updates"
"/mars_pose_node/pose1_cal_state_out"
)

mars_gps_topics=(
"/mars_gps_node/full_state_lite_out"
"/mars_gps_node/full_state_out"
"/mars_gps_node/odom_state_out"
"/mars_gps_node/path_state_out"
"/mars_gps_node/gps1_cal_state_out"
"/mars_gps_node/gps1_enu"
"/mars_gps_node/gps1_pos_in"
"/mars_gps_node/parameter_descriptions"
"/mars_gps_node/parameter_updates"
)

# these topics require the installation of the matrixvision driver
# eg https://github.com/ethz-asl/matrixvision_camera
bluefox_camera_topics=(
"/camera/image_raw"
"/camera/camera_info"
)

# Generate Topic Strings Grouped by Platform Devices (concatinate string arrays)

#-------------------
## Module 1
### Sensors
group_mod1_sensors=(
${mocap_vehicle_topics[@]}
${px4_topics[@]}
${lrf_topic[@]}
${rtk_gps1_topic[@]}
)

### Nodes
group_mod1_nodes=(
${autonomy_topics[@]}
${ms_topics[@]}
${est_topics[@]}
${wd_topics[@]}
${mars_vision_topics[@]}
${mars_dual_topics[@]}
)

# generated comma seperated list, required by nodelet_rosbag in args
printf -v topics_mod1_sensors '%s, ' "${group_mod1_sensors[@]}"
printf -v topics_mod1_nodes '%s, ' "${group_mod1_nodes[@]}"

#-------------------
## Module 2
### Sensors

group_mod2_sensors=(
# ${uwb_topic[@]}
# ${vision_topics[@]}
)

group_mod2_cam=(
${bluefox_camera_topics[@]}
)

# generated comma seperated list, required by nodelet_rosbag in args
printf -v topics_mod2_sensors '%s, ' "${group_mod2_sensors[@]}"
printf -v topics_mod2_cam '%s, ' "${group_mod2_cam[@]}"

#-------------------
## Module Independent
### Calibration Topics (cam+imu)
group_calib=(
${px4_topics[@]}
${bluefox_camera_topics[@]}
)

# generated comma seperated list, required by nodelet_rosbag in args
printf -v topics_calib '%s, ' "${group_calib[@]}"

# Record the given group of topics
echo "Bag Prefix: ${bag_name}"

if [ "${TOPICS}" == "dev1_full" ] ; then
    echo "Recording for device 1 (full): "
    echo "  paths:"
    echo "    local: ${path_local}"
    echo "    - sensors1"
    echo "    - nodes1"
    rosparam dump "${path_local}/${bag_name}_params_$(date +%Y-%m-%d-%H-%M).yaml" & \
    roslaunch nodelet_rosbag nodelet_rosbag.launch \
        start_manager:=True nodelet_manager_name:="record_dev1_manager" nodelet_name:="record_dev1_sensors" \
        rosbag_path:=${path_local} rosbag_prefix:=${bag_name}_mod1_sensors \
        rosbag_topics:="[${topics_mod1_sensors%,}]" & \
    roslaunch nodelet_rosbag nodelet_rosbag.launch \
        start_manager:=False nodelet_manager_name:="record_dev1_manager" nodelet_name:="record_dev1_nodes" \
        rosbag_path:=${path_local} rosbag_prefix:=${bag_name}_mod1_nodes \
        rosbag_topics:="[${topics_mod1_nodes%,}]" && \
    kill $!

elif [ "${TOPICS}" == "dev1_all" ] ; then
    echo "Recording for device 1 (all): "
    echo "  paths:"
    echo "    local: ${path_local}"
    echo "    - sensors1"
    echo "    - nodes1"
    echo "    media: ${path_media}"
    echo "    - cam"
    rosparam dump "${path_local}/${bag_name}_params_$(date +%Y-%m-%d-%H-%M).yaml" & \
    roslaunch nodelet_rosbag nodelet_rosbag.launch \
        start_manager:=True nodelet_manager_name:="record_dev1_manager" nodelet_name:="record_dev1_sensors" \
        rosbag_path:=${path_local} rosbag_prefix:=${bag_name}_mod1_sensors \
        rosbag_topics:="[${topics_mod1_sensors%,}]" & \
    roslaunch nodelet_rosbag nodelet_rosbag.launch \
        start_manager:=${B_DEV_2_START_MANAGER} nodelet_manager_name:="${CAM_NODELET_MANAGER}" nodelet_name:="record_dev1_cams" \
        rosbag_path:=${path_media} rosbag_prefix:=${bag_name}_mod1_cam \
        rosbag_topics:="[${topics_mod2_cam%,}]" & \
    roslaunch nodelet_rosbag nodelet_rosbag.launch \
        start_manager:=False nodelet_manager_name:="record_dev1_manager" nodelet_name:="record_dev1_nodes" \
        rosbag_path:=${path_local} rosbag_prefix:=${bag_name}_mod1_nodes \
        rosbag_topics:="[${topics_mod1_nodes%,}]" && \
    kill $!

elif [ "${TOPICS}" == "dev1_cam" ] ; then
    echo "Recording for device 1 (cam): "
    echo "  paths:"
    echo "    media: ${path_media}"
    echo "    - cam"
    roslaunch nodelet_rosbag nodelet_rosbag.launch \
        start_manager:=${B_DEV_2_START_MANAGER} nodelet_manager_name:="${CAM_NODELET_MANAGER}" nodelet_name:="record_dev2_cams" \
        rosbag_path:=${path_media} rosbag_prefix:=${bag_name}_mod1_cam \
        rosbag_topics:="[${topics_mod2_cam%,}]"

elif [ "${TOPICS}" == "dev1_sensors" ] ; then
	echo "Recording for device 1 (sensors): "
    echo "  paths:"
    echo "    local: ${path_local}"
    echo "    - sensors1"
    roslaunch nodelet_rosbag nodelet_rosbag.launch \
        start_manager:=True nodelet_manager_name:="record_dev1_manager" nodelet_name:="record_dev1_sensors" \
        rosbag_path:=${path_local} rosbag_prefix:=${bag_name}_mod1_sensors \
        rosbag_topics:="[${topics_mod1_sensors%,}]"

elif [ "${TOPICS}" == "dev1_calib" ] ; then
	echo "Recording for device 1 (calib): "
    echo "  paths:"
    echo "    media: ${path_media}"
    echo "    - calib"
    roslaunch nodelet_rosbag nodelet_rosbag.launch \
        start_manager:=True nodelet_manager_name:="record_dev1_manager" nodelet_name:="record_dev1_calib" \
        rosbag_path:=${path_media} rosbag_prefix:=${bag_name}_mod1_calib \
        rosbag_topics:="[${topics_calib%,}]"

elif [ "${TOPICS}" == "dev2_full" ] ; then
    echo "Recording for device 2 (full): "
    echo "  paths:"
    echo "    local: ${path_local}"
    echo "    - sensors2"
    echo "    media: ${path_media}"
    echo "    - cam"
    roslaunch nodelet_rosbag nodelet_rosbag.launch \
        start_manager:=${B_DEV_2_START_MANAGER} nodelet_manager_name:="record_dev2_manager" nodelet_name:="record_dev2_sensors" \
        rosbag_path:=${path_media} rosbag_prefix:=${bag_name}_mod2_sensors \
        rosbag_topics:="[${topics_mod2_sensors%,}]" & \
    roslaunch nodelet_rosbag nodelet_rosbag.launch \
        start_manager:=False nodelet_manager_name:="${CAM_NODELET_MANAGER}" nodelet_name:="record_od2_cams" \
        rosbag_path:=${path_media} rosbag_prefix:=${bag_name}_mod2_cam \
        rosbag_topics:="[${topics_mod2_cam%,}]" && \
    kill $!

elif [ "${TOPICS}" == "dev2_cam" ] ; then
    echo "Recording for device 2 (cam): "
    echo "  paths:"
    echo "    media: ${path_media}"
    echo "    - cam"
    roslaunch nodelet_rosbag nodelet_rosbag.launch \
        start_manager:=${B_DEV_2_START_MANAGER} nodelet_manager_name:="${CAM_NODELET_MANAGER}" nodelet_name:="record_dev2_cams" \
        rosbag_path:=${path_media} rosbag_prefix:=${bag_name}_mod2_cam \
        rosbag_topics:="[${topics_mod2_cam%,}]"

elif [ "${TOPICS}" == "dev2_sensors" ] ; then
    echo "Recording for device 2 (sensors): "
    echo "  paths:"
    echo "    local: ${path_local}"
    echo "    - sensors2"
    roslaunch nodelet_rosbag nodelet_rosbag.launch \
        start_manager:=True nodelet_manager_name:="record_dev2_manager" nodelet_name:="record_dev2_sensors" \
        rosbag_path:=${path_media} rosbag_prefix:=${bag_name}_mod2_sensors \
        rosbag_topics:="[${topics_mod2_sensors%,}]" 

elif [ "${TOPICS}" == "dev2_calib" ] ; then
	echo "Recording for device 2 (calib): "
    echo "  paths:"
    echo "    media: ${path_media}"
    echo "    - calib"
    roslaunch nodelet_rosbag nodelet_rosbag.launch \
        start_manager:=True nodelet_manager_name:="record_dev2_manager" nodelet_name:="record_dev2_calib" \
        rosbag_path:=${path_media} rosbag_prefix:=${bag_name}_mod2_calib \
        rosbag_topics:="[${topics_calib%,}]"

# the following are single entity only and will always use path local
elif [ "${TOPICS}" == "mocap" ] ; then
    echo "Recording MoCap Data: "
    echo "  paths:"
    echo "    local: ${path_local}"
    printf -v topics_mocap '%s, ' "${mocap_vehicle_topics[@]}"
    roslaunch nodelet_rosbag nodelet_rosbag.launch \
        start_manager:=True nodelet_manager_name:="record_manager" nodelet_name:="record_mocap" \
        rosbag_path:=${path_local} rosbag_prefix:=${bag_name}_mocap \
        rosbag_topics:="[${topics_mocap%,}]" 

elif [ "${TOPICS}" == "px4" ] ; then
    echo "Recording MoCap Data: "
    echo "  paths:"
    echo "    local: ${path_local}"
    printf -v topics_px4 '%s, ' "${px4_topics[@]}"
    roslaunch nodelet_rosbag nodelet_rosbag.launch \
        start_manager:=True nodelet_manager_name:="record_manager" nodelet_name:="record_px4" \
        rosbag_path:=${path_local} rosbag_prefix:=${bag_name}_px4 \
        rosbag_topics:="[${topics_px4%,}]"

else # Handle error case
    echo -e "${COL_ERR}[ERROR] The given option '${TOPICS}' is not valid! Please add the group of topics you'd like to record.${NC}"
    exit
fi;
