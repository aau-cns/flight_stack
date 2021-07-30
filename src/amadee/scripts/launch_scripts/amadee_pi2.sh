#!/bin/sh

# perform a RS reset
sudo uhubctl --action cycle --location 2

# Create Tmux Session
CUR_DATE=`date +%F-%H-%M-%S`
export SES_NAME="amadee_pi2_${CUR_DATE}"

## INFO(martin): use .0 .1 if base index has not been configured
tmux new -d -s "${SES_NAME}"

# BACKGROUND
tmux rename-window 'background'
tmux send-keys -t ${SES_NAME}.1 "sleep 10; roslaunch amadee_bringup amaze.launch pi:=2" 'C-m'

tmux attach -t ${SES_NAME}
