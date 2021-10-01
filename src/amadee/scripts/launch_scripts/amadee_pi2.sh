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

    BKG="sleep 10; roslaunch amadee_bringup amaze.launch pi:=2"

else

    BKG="sleep 10; roslaunch amadee_bringup amaze.launch pi:=2 type:=${type}"

fi

# perform a RS reset
sudo uhubctl --action cycle --location 2

# Create Tmux Session
CUR_DATE=`date +%F-%H-%M-%S`
export SES_NAME="amadee_pi2_${CUR_DATE}"

## INFO(martin): use .0 .1 if base index has not been configured
tmux new -d -s "${SES_NAME}"

# BACKGROUND
tmux rename-window 'background'
tmux send-keys -t ${SES_NAME}.1 ${BKG} 'C-m'

tmux attach -t ${SES_NAME}
