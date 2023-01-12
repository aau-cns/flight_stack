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
start_core=true

# script VARIABLES
SLEEP_DURATION_BKG=10
SLEEP_DURATION_EST=10
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
    echo "                  switch between 'pi' or 'xu4'"
    echo "    -d PREFIX     selects the prefix for the launch package, default 'flightstack'"
    echo "    -s PREFIX     selects the prefix for the scripts package, default value of '-d PREFIX'"
    echo "    -f PREFIX     selects the prefix for the launch files, default 'fs'"
    echo "    -a PREFIX     selects the prefix for all of the above"
    echo "                  same as -d PREFIX -s PREFIX -f PREFIX"
    echo ""
    echo "    -c            reserved (by dev_1)"
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
while getopts cvha:d:s:f:t:p: flag
do
    case "${flag}" in
        t) type=${OPTARG};;
        p) PLATFORM=${OPTARG};;
        d) LAUNCH_DIR=${OPTARG};change_scripts=true;;
        f) LAUNCH_PRE=${OPTARG};;
        s) SCRIPTS_DIR=${OPTARG};change_scripts=false;;
        a) LAUNCH_DIR=${OPTARG};LAUNCH_PRE=${OPTARG};SCRIPTS_DIR=${OPTARG};change_scripts=false;;

        c) start_core=false;;
        v) debug_on=true;;
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

# Check if flag is provided and define commands
if [ -z ${type} ]; then

  BKG="sleep ${SLEEP_DURATION_BKG}; roslaunch ${LAUNCH_DIR}_bringup ${LAUNCH_PRE}_sensors.launch dev_id:=2"
  EST="sleep ${SLEEP_DURATION_EST}; roslaunch ${LAUNCH_DIR}_bringup ${LAUNCH_PRE}_estimation.launch dev_id:=2"

elif [ ${type} = "gps" ]; then

  BKG="sleep ${SLEEP_DURATION_BKG}; roslaunch ${LAUNCH_DIR}_bringup ${LAUNCH_PRE}_sensors.launch dev_id:=2"
  EST="sleep ${SLEEP_DURATION_EST}; roslaunch ${LAUNCH_DIR}_bringup ${LAUNCH_PRE}_estimation.launch dev_id:=2 use_gps:=True"

else

  echo "Unknown flight type: '${type}'"
  print_help
  exit 0;

fi

# Create Tmux Session
CUR_DATE=`date +%F-%H-%M-%S`
export SES_NAME="flightstack_dev2_${CUR_DATE}"

## INFO(martin): use .0 .1 if base index has not been configured
tmux new -d -s "${SES_NAME}" -x "$(tput cols)" -y "$(tput lines)"
#-x "$(tput cols)" -y "$(tput lines)"

# BACKGROUND
tmux rename-window 'background'
tmux split-window -v -p 60
tmux send-keys -t ${SES_NAME}.1 "${BKG}" 'C-m'
tmux send-keys -t ${SES_NAME}.2 "${EST}" 'C-m'

tmux attach -t ${SES_NAME}
