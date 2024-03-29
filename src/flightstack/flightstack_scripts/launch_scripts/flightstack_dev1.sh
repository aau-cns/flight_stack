#!/bin/bash

# Copyright (C) 2022 Alessandro Fornaiser, Martin Scheiber,
# and others, Control of Networked Systems, University of Klagenfurt, Austria.
#
# All rights reserved.
#
# This software is licensed under the terms of the BSD-2-Clause-License with
# no commercial use allowed, the full terms of which are made available
# in the LICENSE file. No license in patents is granted.
#
# You can contact the authors at <alessandro.fornasier@ieee.org>,
# and <martin.scheiber@ieee.org>.

################################################################################
# Global Variables                                                             #
################################################################################

script_name="${0}"

# command line flags
debug_on=false
manual_flight=false
dual_platform=false
start_core=true
uart_permission=false

# script VARIABLES
SLEEP_DURATION=10
PLATFORM="pi"
LAUNCH_DIR="flightstack"
SCRIPTS_DIR="flightstack"
LAUNCH_PRE="fs"

################################################################################
# Help                                                                         #
################################################################################

print_help(){
    echo "USAGE: ${script_name} [OPTIONS]"
    echo ""
    echo "  Options:"
    echo "    -t TYPE       executes the type of flight, default 'dh'"
    echo "                  switch between 'gps' or 'dh'"
    echo "    -p PLATFORM   selects the platform for sensors, default 'pi'"
    echo "                  switch between 'pi', 'xu4', 'agx', or 'pc'"
    echo "    -d PREFIX     selects the prefix for the launch package, default 'flightstack'"
    echo "    -s PREFIX     selects the prefix for the scripts package, default value of '-d PREFIX'"
    echo "    -f PREFIX     selects the prefix for the launch files, default 'fs'"
    echo "    -a PREFIX     selects the prefix for all of the above"
    echo "                  same as -d PREFIX -s PREFIX -f PREFIX"
    echo ""
    echo "    -c            disables starting of roscore"
    echo "    -m            'manual flight' - does not autostart autonomy/operator"
    echo "    -n            dual-platform setup (for recording)"
    echo "    -u            change permission for UART device '/dev/DEV_ID' based on platform (-p)"
    echo "    -v            turns debug output on and switches to debug terminal"
    echo ""
    echo "    -h        print this help"
    echo ""
    exit 0;
}

################################################################################
# Execution Options                                                            #
################################################################################

change_scripts=false
# parse flags
while getopts chmvnua:d:f:s:t:p: flag
do
    case "${flag}" in
        t) type=${OPTARG};;
        p) PLATFORM=${OPTARG};;
        d) LAUNCH_DIR=${OPTARG};change_scripts=false;;
        f) LAUNCH_PRE=${OPTARG};;
        s) SCRIPTS_DIR=${OPTARG};change_scripts=false;;
        a) LAUNCH_DIR=${OPTARG};LAUNCH_PRE=${OPTARG};SCRIPTS_DIR=${OPTARG};change_scripts=false;;

        v) debug_on=true;;
        m) manual_flight=true;;
        n) dual_platform=true;;
        c) start_core=false;;
        u) uart_permission=true;;
        h) print_help;;

        *) echo "Unknown option ${flag}"; print_help; exit 1;;
    esac
done
shift $((OPTIND-1))

################################################################################
################################################################################
# MAIN SCRIPT                                                                  #
################################################################################
################################################################################

# source global vars
SOURCE_CMD="$(rospack find ${LAUNCH_DIR}_bringup)/configs/global/${LAUNCH_PRE}_vars.env"
source ${SOURCE_CMD}

# setup sleep cmd
SLEEP_CMD="sleep ${SLEEP_DURATION}"

# check if SCRIPTS_DIR has to be overrriden
if [[ "${change_scripts}" = true ]]; then
  #statements
  SCRIPTS_DIR=${LAUNCH_DIR}
fi

# setup recording args
REC_ADDITIONAL=""
if [[ "${dual_platform}" = true ]]; then
  REC_ADDITIONAL="rec_script_file:=$(rospack find ${SCRIPTS_DIR}_scripts)/record_scripts/record_start_dual.sh rec_cmd:=full"
  REC_ADDITIONAL="${REC_ADDITIONAL} store_script_file:=$(rospack find ${SCRIPTS_DIR}_scripts)/store_scripts/safe_merge_data_multi_dev.sh"
elif [ "${SCRIPTS_DIR}" != "${LAUNCH_DIR}" ]; then
  REC_ADDITIONAL="rec_script_file:=$(rospack find ${SCRIPTS_DIR}_scripts)/record_scripts/record_full.sh"
  REC_ADDITIONAL="${REC_ADDITIONAL} store_script_file:=$(rospack find ${SCRIPTS_DIR}_scripts)/store_scripts/safe_merge_data_single_dev.sh"
fi


# Check if flag is provided and define commands
if [[ -z ${type} || "${type}" = "dh" ]]; then

  SENSOR="roslaunch ${LAUNCH_DIR}_bringup ${LAUNCH_PRE}_sensors.launch dev_id:=1 dev_type:=${PLATFORM}"
  SAFETY="roslaunch ${LAUNCH_DIR}_bringup ${LAUNCH_PRE}_safety.launch dev_id:=1"
  EST="roslaunch ${LAUNCH_DIR}_bringup ${LAUNCH_PRE}_estimation.launch dev_id:=1"
  NAV="roslaunch ${LAUNCH_DIR}_bringup ${LAUNCH_PRE}_navigation.launch dev_id:=1"
  REC="roslaunch ${LAUNCH_DIR}_bringup ${LAUNCH_PRE}_recording.launch dev_id:=1 ${REC_ADDITIONAL}"
  OPS="roslaunch ${LAUNCH_DIR}_bringup ${LAUNCH_PRE}_operator.launch dev_id:=1"

elif [[ "${type}" = "gps" ]]; then

  SENSOR="roslaunch ${LAUNCH_DIR}_bringup ${LAUNCH_PRE}_sensors.launch dev_id:=1 use_gps:=True dev_type:=${PLATFORM}"
  SAFETY="roslaunch ${LAUNCH_DIR}_bringup ${LAUNCH_PRE}_safety.launch dev_id:=1 use_gps:=True"
  EST="roslaunch ${LAUNCH_DIR}_bringup ${LAUNCH_PRE}_estimation.launch dev_id:=1 use_gps:=True"
  NAV="roslaunch ${LAUNCH_DIR}_bringup ${LAUNCH_PRE}_navigation.launch dev_id:=1 use_gps:=True"
  REC="roslaunch ${LAUNCH_DIR}_bringup ${LAUNCH_PRE}_recording.launch dev_id:=1 ${REC_ADDITIONAL}"
  OPS="roslaunch ${LAUNCH_DIR}_bringup ${LAUNCH_PRE}_operator.launch dev_id:=1 estimator_init_service_name:='/mars_gps_node/init_service' 'config_filepath:=\$(find flightstack_bringup)/configs/autonomy/config_gps.yaml'"

else

  echo "Unknown flight type: '${type}'"
  print_help
  exit 0;

fi

# chamge permission for UART serial
if [ "${uart_permission}" = true ]; then
  dev_id=""
  # select dev_id based on platform
  case "${PLATFORM}" in 
    "pi") dev_id="/dev/ttyAMA*";;
    "xu4") dev_id="/dev/ttySAC*";;
    "agx") dev_id="/dev/ttyTHS*";;
    "pc") dev_id="/dev/ttyUSB*";;

    *) echo "Unknown platform ${PLATFORM}"; print_help; exit 1;;
  esac

  sudo chmod 666 ${dev_id}
fi

# Operator
OPMAV="rostopic echo -c /mavros/vision_pose/pose"
OPAUT="rostopic echo -c /autonomy/logger"

# Create Tmux Session
CUR_DATE=`date +%F-%H-%M-%S`
export SES_NAME="flightstack_dev1_${CUR_DATE}"

## INFO(scm): use .0 .1 if base index has not been configured
tmux new -d -s "${SES_NAME}" -x "$(tput cols)" -y "$(tput lines)"

# set base index to 1 (in case different .tmux.conf is used)
tmux set -g -t ${SES_NAME} base-index 1
tmux set -g -t ${SES_NAME} pane-base-index 1

# ROSCORE
tmux rename-window 'roscore_rec'
tmux split-window -v
# source
tmux send-keys -t ${SES_NAME}.1 "source ${SOURCE_CMD}" 'C-m'
tmux send-keys -t ${SES_NAME}.2 "source ${SOURCE_CMD}" 'C-m'
# sleep
tmux send-keys -t ${SES_NAME}.2 "${SLEEP_CMD}" 'C-m'

# roscore
if [[ "${start_core}" = true ]]; then
  tmux send-keys -t ${SES_NAME}.1 "roscore" 'C-m'
else
  tmux send-keys -t ${SES_NAME}.1 "echo 'NOT STARTING ROSOCRE'" 'C-m'
fi
# roslaunch
tmux send-keys -t ${SES_NAME}.2 "${REC}" 'C-m'

# BACKGROUND
tmux new-window -n 'background'
tmux split-window -v
# source
tmux send-keys -t ${SES_NAME}.1 "source ${SOURCE_CMD}" 'C-m'
tmux send-keys -t ${SES_NAME}.2 "source ${SOURCE_CMD}" 'C-m'
# sleep
tmux send-keys -t ${SES_NAME}.1 "${SLEEP_CMD}" 'C-m'
tmux send-keys -t ${SES_NAME}.2 "${SLEEP_CMD}" 'C-m'
# roslaunch
tmux send-keys -t ${SES_NAME}.1 "${SENSOR}" 'C-m'
tmux send-keys -t ${SES_NAME}.2 "${SAFETY}" 'C-m'

# NAVIGATION & CONTROL
tmux new-window -n 'gnc'
tmux split-window -v
# source
tmux send-keys -t ${SES_NAME}.1 "source ${SOURCE_CMD}" 'C-m'
tmux send-keys -t ${SES_NAME}.2 "source ${SOURCE_CMD}" 'C-m'
# sleep
tmux send-keys -t ${SES_NAME}.1 "${SLEEP_CMD}" 'C-m'
tmux send-keys -t ${SES_NAME}.2 "${SLEEP_CMD}" 'C-m'
# roslaunch
tmux send-keys -t ${SES_NAME}.1 "${EST}" 'C-m'
tmux send-keys -t ${SES_NAME}.2 "${NAV}" 'C-m'

# OPERATOR DEBUG WINDOW
if [[ "${debug_on}" = true ]]; then
  tmux new-window -n 'debug'
  tmux split-window -v
  tmux split-window -h
  tmux select-pane -t 1
  tmux split-window -v
  tmux select-pane -t 3
  tmux split-window -v -p 90
  # source
  tmux send-keys -t ${SES_NAME}.1 "source ${SOURCE_CMD}" 'C-m'
  tmux send-keys -t ${SES_NAME}.2 "source ${SOURCE_CMD}" 'C-m'
  tmux send-keys -t ${SES_NAME}.3 "source ${SOURCE_CMD}" 'C-m'
  tmux send-keys -t ${SES_NAME}.4 "source ${SOURCE_CMD}" 'C-m'
  # sleep
  tmux send-keys -t ${SES_NAME}.1 "${SLEEP_CMD}" 'C-m'
  tmux send-keys -t ${SES_NAME}.2 "${SLEEP_CMD}" 'C-m'
  tmux send-keys -t ${SES_NAME}.3 "${SLEEP_CMD}" 'C-m'
  tmux send-keys -t ${SES_NAME}.4 "${SLEEP_CMD}" 'C-m'


  # DEBUG (or operator in manual)
  OP1="rostopic hz -w 100 /camera/camera_info /mavros/imu/data_raw"
  OP2="roscd ${SCRIPTS_DIR}_scripts/"

  # ros stuff
  tmux send-keys -t ${SES_NAME}.1 "${OP1}" 'C-m'
  tmux send-keys -t ${SES_NAME}.2 "${OP2}" 'C-m'
  tmux send-keys -t ${SES_NAME}.5 "${OPMAV}" 'C-m'
  tmux send-keys -t ${SES_NAME}.4 "${OPAUT}" 'C-m'
fi

if [[ "${manual_flight}" != true ]]; then
  # OPERATOR DEBUG WINDOW
  tmux new-window -n 'operator'
  tmux split-window -v -p 90
  # source
  tmux send-keys -t ${SES_NAME}.1 "source ${SOURCE_CMD}" 'C-m'
  tmux send-keys -t ${SES_NAME}.2 "source ${SOURCE_CMD}" 'C-m'
  # sleep
  tmux send-keys -t ${SES_NAME}.1 "${SLEEP_CMD}" 'C-m'
  tmux send-keys -t ${SES_NAME}.2 "${SLEEP_CMD}" 'C-m'


  # DEBUG (or operator in manual)
  OP1="sleep 10; rostopic hz -w 100 /camera/camera_info /mavros/imu/data_raw"
  OP2="roscd ${SCRIPTS_DIR}_scripts/"

  # ros stuff
  tmux send-keys -t ${SES_NAME}.1 "${OP2}" 'C-m'
  tmux send-keys -t ${SES_NAME}.2 "${OPS}" 'C-m'
fi

# select tmux window
if [[ "${debug_on}" = true ]]; then
  tmux select-window -t 'debug'
elif [[ "${manual_flight}" != true ]]; then
  tmux select-window -t 'operator'
else
  tmux select-window -t 'gnc'
fi

tmux attach -t ${SES_NAME}
