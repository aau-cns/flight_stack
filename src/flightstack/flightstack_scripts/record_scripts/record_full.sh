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
COL_DEB='\033[0;32m'  #Green Color
NC='\033[0m'          #No Color

TOPIC_TYPE=dev1_full

# SCRIPT VARIABLES
bag_name="fs"
path_local=""
path_media=""

DEV1_CAM_NODELET_MANAGER="record_dev1_manager"
DEV2_CAM_NODELET_MANAGER="record_dev2_manager"
B_CAMS_START_MANAGER=true

# booleans (flags)
B_USE_ROSBAG_RECORD=false
B_DEBUG_ON=false

################################################################################
# Help                                                                         #
################################################################################
print_help(){
    echo "USAGE: ${script_name} <TOPICS> [OPTIONS]"
    echo ""
    echo "  Topics:           details the topics to record, choose one from below"
    echo "    dev<N>_all      all rostopics typically on dev <N>"
    echo "    dev<N>_full     all rostopics typically on dev <N>, excluding cams"
    echo "    dev<N>_sensors  all sensor-related rostopics typically on dev <N>"
    echo "    dev<N>_nodes    all nodes-related rostopics typically on dev <N>"
    echo "    dev<N>_cams     all cams-related rostopics typically on dev <N>"
    echo "    , with <N> either '1' or '2'."
    echo ""
    echo "  Options:"
    echo "    -l PATH         path to internal media"
    echo "    -m PATH         path to external media device (for images)"
    echo "    -p NAME         prefix of bagfile, default 'fs'"
    echo ""
    echo "    -b              use 'rosbag record' instead of the flight stack's"
    echo "                      'nodlet recorder'"
    echo "    -c              use 'nodelet_manager' for standard recording settings"
    echo "    -v              enable detailed debug output"
    echo ""
    echo "    -h          print this help"
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
while getopts hvbcl:m:p: flag
do
  case "${flag}" in
    l) path_local=${OPTARG};;
    m) path_media=${OPTARG};;
    p) bag_name=${OPTARG};;

    c)  DEV1_CAM_NODELET_MANAGER="nodelet_manager";
        DEV2_CAM_NODELET_MANAGER="nodelet_manager";
        B_CAMS_START_MANAGER=false;;
    
    b) B_USE_ROSBAG_RECORD=true;;
    v) B_DEBUG_ON=true;;
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

################################################################################
# SETUP TOPIC GROUPS                                                           #
################################################################################

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
"/${FS_OPTITRACK_OBJECT_NAME}/vrpn_client/raw_transform"
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

mars_topics=(
"/${FS_ESTIMATOR_NODE_NAME}/full_state_lite_out"
"/${FS_ESTIMATOR_NODE_NAME}/full_state_out"
"/${FS_ESTIMATOR_NODE_NAME}/pose_state_out"
"/${FS_ESTIMATOR_NODE_NAME}/odom_state_out"
"/${FS_ESTIMATOR_NODE_NAME}/path_state_out"
"/${FS_ESTIMATOR_NODE_NAME}/gps1_cal_state_out"
"/${FS_ESTIMATOR_NODE_NAME}/gps1_enu"
"/${FS_ESTIMATOR_NODE_NAME}/gps1_pos_in"
"/${FS_ESTIMATOR_NODE_NAME}/pose1_in"
"/${FS_ESTIMATOR_NODE_NAME}/pose1_cal_state_out"
"/${FS_ESTIMATOR_NODE_NAME}/parameter_descriptions"
"/${FS_ESTIMATOR_NODE_NAME}/parameter_updates"
)

# Generate Topic Strings Grouped by Platform Devices (concatinate string arrays)

#-------------------
## Module 1
### Sensors
group_mod1_sensors=(
${mocap_vehicle_topics[@]}
${px4_topics[@]}
${FS_RECORD_ADD_DEV1_SENSOR[@]}
)

### Nodes
group_mod1_nodes=(
${autonomy_topics[@]}
${ms_topics[@]}
${est_topics[@]}
${wd_topics[@]}
${mars_topics[@]}
${FS_RECORD_ADD_DEV1_ESTIMATOR[@]}
)

### Cameras
group_mod1_cams=(
${FS_RECORD_ADD_DEV1_CAM[@]}
)


# generate final list of topics for recording - dev1
if [ ${B_USE_ROSBAG_RECORD} == true ]; then
  topics_mod1_sensors=${group_mod1_sensors[@]}
  topics_mod1_nodes=${group_mod1_nodes[@]}
  topics_mod1_cams=${group_mod1_cams[@]}
else
  # generated comma seperated list, required by nodelet_rosbag in args
  printf -v topics_mod1_sensors '%s,' "${group_mod1_sensors[@]}"
  printf -v topics_mod1_nodes '%s,' "${group_mod1_nodes[@]}"
  printf -v topics_mod1_cams '%s,' "${group_mod1_cams[@]}"
fi

#-------------------
## Module 2

### Sensors
group_mod2_sensors=(
${FS_RECORD_ADD_DEV2_SENSOR[@]}
)

### Nodes
group_mod2_nodes=(
${FS_RECORD_ADD_DEV2_EST[@]}
)

### Cameras
group_mod2_cams=(
${FS_RECORD_ADD_DEV2_CAM[@]}
)

# generate final list of topics for recording - dev2
if [ ${B_USE_ROSBAG_RECORD} == true ]; then
  topics_mod2_sensors=${group_mod2_sensors[@]}
  topics_mod2_nodes=${group_mod2_nodes[@]}
  topics_mod2_cams=${group_mod2_cams[@]}
else
  # generated comma seperated list, required by nodelet_rosbag in args
  printf -v topics_mod2_sensors '%s,' "${group_mod2_sensors[@]}"
  printf -v topics_mod2_nodes '%s,' "${group_mod2_nodes[@]}"
  printf -v topics_mod2_cams '%s,' "${group_mod2_cams[@]}"
fi


#-------------------
## Module Independent
### Calibration Topics (cam+imu)
group_calib=(
${mocap_vehicle_topics[@]}
${px4_topics[@]}
${FS_RECORD_ADD_DEV1_SENSOR[@]}
${FS_RECORD_ADD_DEV1_CAM[@]}
)

# generate final list of topics for recording - calib
if [ ${B_USE_ROSBAG_RECORD} == true ]; then
  topics_calib=${group_calib[@]}
else
  # generated comma seperated list, required by nodelet_rosbag in args
  printf -v topics_calib '%s,' "${group_calib[@]}"
fi

################################################################################
################################################################################
# MAIN SCRIPT                                                                  #
################################################################################
################################################################################

if [ ${B_DEBUG_ON} = true ]; then
  set +x
fi

# setup modules to record
b_record_dev1_sensors=false
b_record_dev1_nodes=false
b_record_dev1_cams=false
b_record_dev2_sensors=false
b_record_dev2_nodes=false
b_record_dev2_cams=false
b_additional_check=false
case ${TOPICS} in
  dev1_all)
    b_record_dev1_sensors=true;
    b_record_dev1_nodes=true;
    b_record_dev1_cams=true;;
  dev2_all)
    b_record_dev2_sensors=true;
    b_record_dev2_nodes=true;
    b_record_dev2_cams=true;;

  dev1_full)
    b_record_dev1_sensors=true;
    b_record_dev1_nodes=true;;
  dev2_full)
    b_record_dev2_sensors=true;
    b_record_dev2_nodes=true;;

  dev1_sensors)
    b_record_dev1_sensors=true;;
  dev2_sensors)
    b_record_dev2_sensors=true;;

  dev1_nodes)
    b_record_dev1_nodes=true;;
  dev2_nodes)
    b_record_dev2_nodes=true;;

  dev1_cams)
    b_record_dev1_cams=true;;
  dev2_cams)
    b_record_dev2_cams=true;;

  all)
    b_record_dev1_sensors=true;
    b_record_dev1_nodes=true;
    b_record_dev1_cams=true;
    b_record_dev2_sensors=true;
    b_record_dev2_nodes=true;
    b_record_dev2_cams=true;;

  *)
    # special configurations
    b_additional_check=true;;
esac

################################################################################
# RECORDING                                                                    #
################################################################################

RECORD_CMD_ARR=()
VERBOSE_LOCAL=""
VERBOSE_MEDIA=""

# first check if special recording options were selected
if [ ${b_additional_check} == true ]; then
  if [ "${TOPICS}" == "calib" ]; then
    VERBOSE_MEDIA="${VERBOSE_MEDIA}\n      - sensors1"
    VERBOSE_MEDIA="${VERBOSE_MEDIA}\n      - cams1"
    if [ ${B_USE_ROSBAG_RECORD} == true ]; then
      RECORD_CMD_ARR+=( "rosbag record 
        --tcpnodelay -b 512 --split --size=1024
        -o ${path_local}/${bag_name}_calib
        ${topics_calib}" )
    else
      RECORD_CMD_ARR+=( "roslaunch nodelet_rosbag nodelet_rosbag.launch
        start_manager:=${B_CAMS_START_MANAGER} 
        nodelet_manager_name:=${DEV1_CAM_NODELET_MANAGER}
        nodelet_name:=record_calib
        rosbag_path:=${path_local} rosbag_prefix:=${bag_name}_calib
        rosbag_topics:=[${topics_calib%,}]" )
    fi
  
  elif [ "${TOPICS}" == "px4" ]; then
    VERBOSE_LOCAL="${VERBOSE_LOCAL}\n      - px4"
    if [ ${B_USE_ROSBAG_RECORD} == true ]; then
      topics_px4=${px4_topics[@]}
      RECORD_CMD_ARR+=( "rosbag record --tcpnodelay -b 512 --split --size=512
        -o ${path_local}/${bag_name}_px4
        ${topics_px4}" )
    else
      printf -v topics_px4 '%s,' "${px4_topics[@]}"
      RECORD_CMD_ARR+=( "roslaunch nodelet_rosbag nodelet_rosbag.launch 
        start_manager:=True
        nodelet_manager_name:=record_manager
        nodelet_name:=record_px4
        rosbag_path:=${path_local} rosbag_prefix:=${bag_name}_px4
        rosbag_topics:=[${topics_px4%,}]" )
    fi
  
  elif [ "${TOPICS}" == "mocap" ]; then
    VERBOSE_LOCAL="${VERBOSE_LOCAL}\n      - mocap"
    if [ ${B_USE_ROSBAG_RECORD} == true ]; then
      topics_mocap=${mocap_vehicle_topics[@]}
      RECORD_CMD_ARR+=( "rosbag record --tcpnodelay -b 512 --split --size=512
        -o ${path_local}/${bag_name}_mocap
        ${topics_mocap}" )
    else
      printf -v topics_mocap '%s,' "${mocap_vehicle_topics[@]}"
      RECORD_CMD_ARR+=( "roslaunch nodelet_rosbag nodelet_rosbag.launch 
        start_manager:=True
        nodelet_manager_name:=record_manager
        nodelet_name:=record_mocap
        rosbag_path:=${path_local} rosbag_prefix:=${bag_name}_mocap
        rosbag_topics:=[${topics_mocap%,}]" )
    fi

  else
    # option does not exist, thus throw error
    echo -e "${COL_ERR}[ERROR] The given option '${TOPICS}' is not valid!"
    echo -e "        Please add the group of topics you'd like to record.${NC}"
    print_help
    exit 1;
  fi

# in case of non-special recordings
else
  # setup start manager vars
  start_manager1=true
  start_manager2=true

  # check each setting individually, first local stuff
  if [ ${b_record_dev1_sensors} == true ]; then
    VERBOSE_LOCAL="${VERBOSE_LOCAL}\n      - sensors1"
    if [ ${B_USE_ROSBAG_RECORD} == true ]; then
      RECORD_CMD_ARR+=( "rosbag record --tcpnodelay -b 512 --split --size=512
        -o ${path_local}/${bag_name}_mod1_sensors
        ${topics_mod1_sensors}" )
    else
      RECORD_CMD_ARR+=( "roslaunch nodelet_rosbag nodelet_rosbag.launch
        start_manager:=${start_manager1} nodelet_manager_name:=record_dev1_manager nodelet_name:=record_dev1_sensors
        rosbag_path:=${path_local} rosbag_prefix:=${bag_name}_mod1_sensors
        rosbag_topics:=[${topics_mod1_sensors%,}]" )
      start_manager1=false 
    fi
  fi

  if [ ${b_record_dev1_nodes} == true ]; then
    VERBOSE_LOCAL="${VERBOSE_LOCAL}\n      - nodes1"
    if [ ${B_USE_ROSBAG_RECORD} == true ]; then
      RECORD_CMD_ARR+=( "rosbag record --tcpnodelay -b 512 --split --size=512
        -o ${path_local}/${bag_name}_mod1_nodes
        ${topics_mod1_nodes}" )
    else
      RECORD_CMD_ARR+=( "roslaunch nodelet_rosbag nodelet_rosbag.launch
        start_manager:=${start_manager1} nodelet_manager_name:=record_dev1_manager nodelet_name:=record_dev1_nodes
        rosbag_path:=${path_local} rosbag_prefix:=${bag_name}_mod1_nodes
        rosbag_topics:=[${topics_mod1_nodes%,}]" )
      start_manager1=false
    fi
  fi

  if [ ${b_record_dev2_sensors} == true ]; then
    VERBOSE_LOCAL="${VERBOSE_LOCAL}\n      - sensors2"
    if [ ${B_USE_ROSBAG_RECORD} == true ]; then
      RECORD_CMD_ARR+=( "rosbag record --tcpnodelay -b 512 --split --size=512
        -o ${path_local}/${bag_name}_mod2_sensors
        ${topics_mod2_sensors}" )
    else
      RECORD_CMD_ARR+=( "roslaunch nodelet_rosbag nodelet_rosbag.launch
        start_manager:=${start_manager2} nodelet_manager_name:=record_dev2_manager nodelet_name:=record_dev2_sensors
        rosbag_path:=${path_local} rosbag_prefix:=${bag_name}_mod2_sensors
        rosbag_topics:=[${topics_mod2_sensors%,}]" )
      start_manager2=false
    fi
  fi

  if [ ${b_record_dev2_nodes} == true ]; then
    VERBOSE_LOCAL="${VERBOSE_LOCAL}\n      - nodes2"
    if [ ${B_USE_ROSBAG_RECORD} == true ]; then
      RECORD_CMD_ARR+=( "rosbag record --tcpnodelay -b 512 --split --size=512
        -o ${path_local}/${bag_name}_mod2_nodes
        ${topics_mod2_nodes}" )
    else
      RECORD_CMD_ARR+=( "roslaunch nodelet_rosbag nodelet_rosbag.launch
        start_manager:=${start_manager2} nodelet_manager_name:=record_dev2_manager nodelet_name:=record_dev2_nodes
        rosbag_path:=${path_local} rosbag_prefix:=${bag_name}_mod2_nodes
        rosbag_topics:=[${topics_mod2_nodes%,}]" )
      start_manager2=false
    fi
  fi

  # media
  if [ ${b_record_dev1_cams} == true ]; then
    VERBOSE_MEDIA="${VERBOSE_MEDIA}\n      - cams1"
    # INFO(scm): cams alwyas uses nodelet manager, as this improves a lot
    RECORD_CMD_ARR+=( "roslaunch nodelet_rosbag nodelet_rosbag.launch 
      start_manager:=${B_CAMS_START_MANAGER}
      nodelet_manager_name:=${DEV1_CAM_NODELET_MANAGER}
      nodelet_name:=record_dev1_cams
      rosbag_path:=${path_local} rosbag_prefix:=${bag_name}_mod1_cams
      rosbag_topics:=[${topics_mod1_cams%,}]" )
  fi

  if [ ${b_record_dev2_cams} == true ]; then
    VERBOSE_MEDIA="${VERBOSE_MEDIA}\n      - cams2"
    # INFO(scm): cams alwyas uses nodelet manager, as this improves a lot
    RECORD_CMD_ARR+=( "roslaunch nodelet_rosbag nodelet_rosbag.launch
      start_manager:=${B_CAMS_START_MANAGER}
      nodelet_manager_name:=${DEV2_CAM_NODELET_MANAGER}
      nodelet_name:=record_dev2_cams
      rosbag_path:=${path_local} rosbag_prefix:=${bag_name}_mod2_cams
      rosbag_topics:=[${topics_mod2_cams%,}]" )
  fi
fi

# print path output
echo "Recording (${TOPICS}): "
echo "  paths:"
echo -e "    local: ${path_local}${VERBOSE_LOCAL}"
echo -e "    media: ${path_media}${VERBOSE_MEDIA}"
echo ""

# execute recording in bkg
for cmd in "${RECORD_CMD_ARR[@]}"; do
  if [ ${B_DEBUG_ON} == true ]; then
    echo -e "${COL_DEB}executing: ${cmd}${NC}"
  fi
  ${cmd} &
done

if [ ${B_DEBUG_ON} = true ]; then
  set -x
fi

exit 0;
