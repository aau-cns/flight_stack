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

  SENSOR="sleep 10; roslaunch flightstack_bringup fs_sensors.launch dev_id:=1"
  SAFETY="sleep 10; roslaunch flightstack_bringup fs_safety.launch dev_id:=1"
  EST="sleep 10; roslaunch flightstack_bringup fs_estimation.launch dev_id:=1"
  NAV="sleep 10; roslaunch flightstack_bringup fs_navigation.launch dev_id:=1"
  REC="sleep 10; roslaunch flightstack_bringup fs_recording.launch dev_id:=1"
  DROP="sleep 10; roslaunch flightstack_bringup topic_dropper.launch"

elif [ "${type}" = "dualpose_gps" ]; then

  SENSOR="sleep 10; roslaunch flightstack_bringup fs_sensors.launch dev_id:=1 use_optitrack:=False"
  SAFETY="sleep 10; roslaunch flightstack_bringup fs_safety.launch dev_id:=1 dronehall:=False"
  EST="sleep 10; roslaunch flightstack_bringup fs_estimation.launch dev_id:=1 dual_pose:=True use_gps:=True"
  NAV="sleep 10; roslaunch flightstack_bringup fs_navigation.launch dev_id:=1"
  REC="sleep 10; roslaunch flightstack_bringup fs_recording.launch dev_id:=1"
  DROP="sleep 10; roslaunch flightstack_bringup topic_dropper.launch"

elif [ "${type}" = "gps" ]; then

  echo "TODO"
  exit

fi


# Operator
OPLRF="sleep 10; rostopic echo -c /lidar_lite/range/range"
OPMAV="sleep 10; rostopic echo -c /mavros/vision_pose/pose"
OPAUT="sleep 1-; rostopic echo -c /autonomy/logger"

# Create Tmux Session
CUR_DATE=`date +%F-%H-%M-%S`
export SES_NAME="flightstack_dev1_${CUR_DATE}"

## INFO(martin): use .0 .1 if base index has not been configured
tmux new -d -s "${SES_NAME}" -x "$(tput cols)" -y "$(tput lines)"

# ROSCORE
tmux rename-window 'roscore_rec'
tmux split-window -v
tmux send-keys -t ${SES_NAME}.1 "roscore" 'C-m'
tmux send-keys -t ${SES_NAME}.2 "${REC}" 'C-m'

# BACKGROUND
tmux new-window -n 'background'
tmux split-window -v
tmux split-window -v
tmux send-keys -t ${SES_NAME}.1 "${SENSOR}" 'C-m'
tmux send-keys -t ${SES_NAME}.2 "${SAFETY}" 'C-m'
tmux send-keys -t ${SES_NAME}.3 "${DROP}" 'C-m'

# NAVIGATION & CONTROL
tmux new-window -n 'gnc'
tmux split-window -v
tmux send-keys -t ${SES_NAME}.1 "${EST}" 'C-m'
tmux send-keys -t ${SES_NAME}.2 "${NAV}" 'C-m'

# OPERATOR DEBUG WINDOW
tmux new-window -n 'debug'
tmux split-window -v
tmux split-window -h
tmux select-pane -t 1
tmux split-window -v
tmux select-pane -t 3
tmux split-window -v -p 90


# DEBUG (or operator in manual)
OP1="sleep 10; rostopic hz -w 100 /camera/camera_info /mavros/imu/data_raw"
OP2="roscd flightstack_scripts/system_scripts"

tmux send-keys -t ${SES_NAME}.1 "${OP1}" 'C-m'
tmux send-keys -t ${SES_NAME}.2 "${OP2}" 'C-m'
tmux send-keys -t ${SES_NAME}.5 "${OPMAV}" 'C-m'
tmux send-keys -t ${SES_NAME}.3 "${OPLRF}" 'C-m'
tmux send-keys -t ${SES_NAME}.4 "${OPAUT}" 'C-m'

tmux select-window -t 'operator'
tmux attach -t ${SES_NAME}
