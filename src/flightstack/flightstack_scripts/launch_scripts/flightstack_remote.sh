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
automatic_routing=true

# script VARIABLES
SLEEP_DURATION=10
PLATFORM="pi"
LOGIN_DEV1="${1}"
LOGIN_DEV2=

################################################################################
# Help                                                                         #
################################################################################

print_help(){
  echo "USAGE: ${script_name} LOGIN_DEV1 [OPTIONS]"
  echo ""
  echo "  Options:"
  echo "    -t TYPE       executes the type of flight, default 'dh'"
  echo "                  switch between 'gps' or 'dh'"
  echo "    -p PLATFORM   selects the platform for sensors, default 'pi'"
  echo "                  switch between 'pi' or 'xu4'"
  echo "    -n LOGIN_DEV2 dual-platform setup (for recording)"
  echo "    -d            turns debug output on and switches to debug terminal"
  echo "    -r            deactivate automatic routing"
  echo ""
  echo "    -h        print this help"
  echo ""
  exit 0;
}

################################################################################
# Execution Options                                                            #
################################################################################
# check if login information was provided
if [[ -z "${LOGIN_DEV1}" ]]; then
  echo "[ERROR] Login information for dev1 required."
  print_help
  exit 0;
fi
# shift index to second place for optional arguments
OPTIND=2

# parse flags
while getopts dhrn:t:p: flag
do
  case "${flag}" in
    t) type=${OPTARG};;
    p) PLATFORM=${OPTARG};;
    n) LOGIN_DEV2=${OPTARG};;

    d) debug_on=true;;
    r) automatic_routing=false;;
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

# check if connection to targets work
ssh -q ${LOGIN_DEV1} exit
if [ $? != 0 ]; then
  echo "[ERROR] No connection to ${LOGIN_DEV1}."
  exit 0;
fi
if [[ ! -z ${LOGIN_DEV2} ]]; then
  ssh -q ${LOGIN_DEV2} exit
  if [ $? != 0  ]; then
    echo "[ERROR] No connection to ${LOGIN_DEV2}."
    #exit 0;
  fi
fi

# in multiplatform the flightstack is configured to have shared ethernet connection
# thus setup automatic routing
if [[ "${automatic_routing}" = true && ! -z ${LOGIN_DEV2} ]]; then
  # check if LOGIN_DEV1 contains @
  if [[ ${LOGIN_DEV1} == *"@"* ]]; then
    DEV1_IP=$(echo ${LOGIN_DEV1} | tr '@' '\n' | sed -n 2p)
  # check if LOGIN_DEV1 is saved under known hosts
  elif [[ $(ssh -G ${LOGIN_DEV1} | awk '$1 == "hostname" { print $2 }') != ${LOGIN_DEV1} ]]; then
    DEV1_IP=$(ssh -G ${LOGIN_DEV1} | awk '$1 == "hostname" { print $2 }')
  else
    echo "[ERROR] Cannot derive IP to connect to ${LOGIN_DEV1}"
    echo "        if this error persists, please deactivate autorouting with option '-r'."
    exit 0
  fi

  # make sure DEV1_IP is really an ip and not webadress, e.g. google.com
  DEV1_IP=$(getent ahosts "${DEV1_IP}" | awk '{print $1; exit}')
  CONN_NAME=$(ip route get "${DEV1_IP}" | grep -Po '(?<=(dev ))(\S+)')
  export ROUTE_CMD="sudo route add -net 10.42.0.0 netmask 255.255.255.0 dev ${CONN_NAME}"
else
  export ROUTE_CMD=""
fi
# INFO(scm): not adding route command for now, as this requires some testing and setups on SKIFF config, to ensure that dev1 uses local 10.42.0.x IP in ros_env.sh
if [[ "${debug_on}" = true ]]; then
  echo "use the following command for setting ROS route correctly:"
  echo "${ROUTE_CMD}"
  sleep 5;
fi

# setup recording args
CMD_ADDITIONAL=""
if [[ "${debug_on}" = true ]]; then
  CMD_ADDITIONAL="-d"
fi

# Check if flag is provided and define commands
if [ -z ${type} ]; then

  CMD1="ssh -t ${LOGIN_DEV1} 'bash -c \"fs_dev1 ${CMD_ADDITIONAL}\"'"
  CMD2="ssh -t ${LOGIN_DEV2} 'bash -c \"fs_dev2 ${CMD_ADDITIONAL}\"'"

else

  CMD1="ssh -t ${LOGIN_DEV1} 'bash -c \"fs_dev1 -t ${type} ${CMD_ADDITIONAL}\"'"
  CMD2="ssh -t ${LOGIN_DEV2} 'bash -c \"fs_dev2 -t ${type} ${CMD_ADDITIONAL}\"'"

fi

# Create Tmux Session
CUR_DATE=`date +%F-%H-%M-%S`
export SES_NAME="flightstack_remote_${CUR_DATE}"

## INFO(scm): use .0 .1 if base index has not been configured
tmux new -d -s "${SES_NAME}"

# ROSCORE & OPERATOR
tmux rename-window 'dev1'
tmux send-keys -t ${SES_NAME}.1 "${CMD1}" 'C-m'

if [[ ! -z ${LOGIN_DEV2} ]]; then
  # BACKGROUND
  tmux new-window -n 'dev2'
  tmux send-keys -t ${SES_NAME}.1 "${CMD2}" 'C-m'
fi

tmux select-window -t 'dev1'
tmux attach -t ${SES_NAME}
