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

  OPS="sleep 10; roslaunch flightstack_bringup fs_operator.launch dev_id:=1"

elif [ "${type}" = "dualpose_gps" ]; then

  OPS="sleep 10; roslaunch flightstack_bringup fs_operator.launch dev_id:=1 estimator_init_service_name:='/mars_dualpose_node/init_service' 'config_filepath:=\$(find flightstack_bringup)/configs/autonomy/config_gps.yaml'"

elif [ "${type}" = "vision_gps" ]; then

  OPS="sleep 10; roslaunch flightstack_bringup fs_operator.launch dev_id:=1 estimator_init_service_name:='/mars_gps_vision_node/init_service' 'config_filepath:=\$(find flightstack_bringup)/configs/autonomy/config_gps.yaml'"

elif [ "${type}" = "vision_rtk" ]; then

  OPS="sleep 10; roslaunch flightstack_bringup fs_operator.launch dev_id:=1 estimator_init_service_name:='/mars_gps_vision_node/init_service' 'config_filepath:=\$(find flightstack_bringup)/configs/autonomy/config_rtk.yaml'"

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
export SES_NAME="flightstack_ops_${CUR_DATE}"

## INFO(martin): use .0 .1 if base index has not been configured
tmux new -d -s "${SES_NAME}" -x "$(tput cols)" -y "$(tput lines)"


# OPERATOR DEBUG WINDOW
tmux new-window -n 'operator'
tmux split-window -v -p 90


# DEBUG (or operator in manual)
OP1="sleep 10; rostopic hz -w 100 /camera/camera_info /mavros/imu/data_raw"
OP2="roscd flightstack_scripts/system_scripts"

tmux send-keys -t ${SES_NAME}.1 "${OP2}" 'C-m'
tmux send-keys -t ${SES_NAME}.2 "${OPS}" 'C-m'

tmux select-window -t 'operator'
tmux attach -t ${SES_NAME}
