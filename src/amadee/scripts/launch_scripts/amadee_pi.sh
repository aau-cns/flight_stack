#!/bin/sh

# Create Tmux Session
CUR_DATE=`date +%F-%H-%M-%S`
export SES_NAME="amadee_pi1_${CUR_DATE}"

## INFO(martin): use .0 .1 if base index has not been configured
tmux new -d -s "${SES_NAME}"

# ROSCORE
tmux rename-window 'roscore'
tmux send-keys -t ${SES_NAME}.1 "roscore" 'C-m'

# BACKGROUND
tmux new-window -n 'background'
tmux send-keys -t ${SES_NAME}.1 "roslaunch amadee_bringup amaze.launch pi:=1" 'C-m'

# BACKGROUND
tmux new-window -n 'operator'
tmux send-keys -t ${SES_NAME}.1 "roslaunch amadee_bringup amadee_operator pi:=1" 'C-m'

tmux attach -t ${SES_NAME}
