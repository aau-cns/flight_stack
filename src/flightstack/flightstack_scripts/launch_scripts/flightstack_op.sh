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

# script VARIABLES
SLEEP_DURATION=10
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
    echo "    -d PREFIX     selects the prefix for the launch package, default 'flightstack'"
    echo "    -s PREFIX     selects the prefix for the scripts package, default value of '-d PREFIX'"
    echo "    -f PREFIX     selects the prefix for the launch files, default 'fs'"
    echo "    -a PREFIX     selects the prefix for all of the above"
    echo "                  same as -d PREFIX -s PREFIX -f PREFIX"
    echo ""
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
while getopts vha:s:d:f:t: flag
do
    case "${flag}" in
        t) type=${OPTARG};;
        d) LAUNCH_DIR=${OPTARG};change_scripts=true;;
        f) LAUNCH_PRE=${OPTARG};;
        s) SCRIPTS_DIR=${OPTARG};change_scripts=false;;
        a) LAUNCH_DIR=${OPTARG};LAUNCH_PRE=${OPTARG};SCRIPTS_DIR=${OPTARG};change_scripts=false;;

        d) debug_on=true;;
        h) print_help;;

        *) echo "Unknown option ${flag}"; print_help;;
    esac
done
shift $((OPTIND-1))

################################################################################
################################################################################
# MAIN SCRIPT                                                                  #
################################################################################
################################################################################

# check if SCRIPTS_DIR has to be overrriden
if [[ "${change_scripts}" = true ]]; then
  #statements
  SCRIPTS_DIR=${LAUNCH_DIR}
fi

# Check if flag is provided and define commands
if [ -z ${type} ]; then

  OPS="sleep ${SLEEP_DURATION}; roslaunch ${LAUNCH_DIR}_bringup ${LAUNCH_PRE}_operator.launch dev_id:=1"

elif [ "${type}" = "gps" ]; then

  OPS="sleep ${SLEEP_DURATION}; roslaunch ${LAUNCH_DIR}_bringup ${LAUNCH_PRE}_operator.launch dev_id:=1 estimator_init_service_name:='/mars_gps_node/init_service' 'config_filepath:=\$(find ${LAUNCH_DIR}_bringup)/configs/autonomy/config_gps.yaml'"

else

  echo "Unknown flight type: '${type}'"
  print_help
  exit 0;

fi


# Operator
OPLRF="sleep 10; rostopic echo -c /lidar_lite/range/range"
OPMAV="sleep 10; rostopic echo -c /mavros/vision_pose/pose"
OPAUT="sleep 1-; rostopic echo -c /autonomy/logger"

# Create Tmux Session
CUR_DATE=`date +%F-%H-%M-%S`
export SES_NAME="flightstack_ops_${CUR_DATE}"

## INFO(martin): use .0 .1 if base index has not been configured
tmux new -d -s "${SES_NAME}" -x "$(tput cols)" -y "$(tput lines)"


# OPERATOR DEBUG WINDOW
tmux rename-window 'operator'
# tmux split-window -v -p 90


# DEBUG (or operator in manual)
OP1="sleep 10; rostopic hz -w 100 /camera/camera_info /mavros/imu/data_raw"
OP2="roscd ${SCRIPTS_DIR}_scripts/system_scripts"

# tmux send-keys -t ${SES_NAME}.1 "${OP2}" 'C-m'
# tmux send-keys -t ${SES_NAME}.2 "${OPS}" 'C-m'
tmux send-keys -t ${SES_NAME}.1 "${OPS}" 'C-m'

# select tmux window
# if [[ "${debug_on}" = true ]]; then
  # tmux select-window -t 'debug'
# else
  tmux select-window -t 'operator'
# fi

tmux attach -t ${SES_NAME}
