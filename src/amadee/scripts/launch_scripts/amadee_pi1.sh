#!/bin/sh

# parse flags
while getopts t: flag
do
    case "${flag}" in
        t) type=${OPTARG};;
    esac
done
shift $((OPTIND-1))

# Check if flag is provided and define commands
if [ -z ${type} ]; then

    BKG="sleep 10; roslaunch amadee_bringup amaze.launch pi:=1"
    OPR="sleep 10; roslaunch amadee_bringup amadee_operator.launch pi:=1"

else

    BKG="sleep 10; roslaunch amadee_bringup amaze.launch pi:=1 type:=${type}"
    OPR="sleep 10; roslaunch amadee_bringup amadee_operator.launch pi:=1 type:=${type}"

fi

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
tmux send-keys -t ${SES_NAME}.1 ${BKG} 'C-m'

# OPERATOR
tmux new-window -n 'operator'
tmux send-keys -t ${SES_NAME}.1 ${OPR} 'C-m'

tmux attach -t ${SES_NAME}
