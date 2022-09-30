#!/bin/sh

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


# parse flags
while getopts t: flag
do
    case "${flag}" in
        t) type=${OPTARG};;
    # case "${flag}" in
    #     i) illu=${OPTARG};;
    esac
done
shift $((OPTIND-1))



# Check if flag is provided and define commands
if [ -z ${type} ]; then

  BKG="sleep 10; roslaunch flightstack_bringup fs_sensors.launch dev_id:=2"
  EST="sleep 20; roslaunch flightstack_bringup fs_estimation.launch dev_id:=2"

elif [ ${type} = "gps" ]; then

  BKG="sleep 10; roslaunch flightstack_bringup fs_sensors.launch dev_id:=2"
  EST="sleep 20; roslaunch flightstack_bringup fs_estimation.launch dev_id:=2 use_gps:=True"

fi

# chmod for UWB
sudo chmod 666 /dev/ttyACM0

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
