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

  BKG="sleep 10; roslaunch flightstack_bringup fs_sensors.launch dev_id:=2"
  EST="sleep 20; roslaunch bw2_ms_msckf bw2.launch global_inti_only:=True"

elif [ ${type} = "dualpose_gps" ]; then

  BKG="sleep 10; roslaunch flightstack_bringup fs_sensors.launch dev_id:=2 shutter:=0.0005"
  EST="sleep 20; roslaunch bw2_ms_msckf bw2.launch global_inti_only:=True"

fi

# chmod for UWB
sudo chmod 666 /dev/ttyACM0

# Create Tmux Session
CUR_DATE=`date +%F-%H-%M-%S`
export SES_NAME="flightstack_dev2_${CUR_DATE}"

## INFO(martin): use .0 .1 if base index has not been configured
tmux new -d -s "${SES_NAME}" -x "$(tput cols)" -y "$(tput lines)"
#-x "$(tput cols)" -y "$(tput lines)"

OP1="sleep 10; rostopic echo -c /bw2_ms_msckf/pose/pose/position"
OP2="sleep 10; rostopic echo -c /bw2_ms_msckf/pose/pose/rotation"

# BACKGROUND
tmux rename-window 'background'
tmux split-window -v -p 60
tmux split-window -v -p 20
tmux send-keys -t ${SES_NAME}.1 "${BKG}" 'C-m'
tmux send-keys -t ${SES_NAME}.2 "${EST}" 'C-m'
tmux send-keys -t ${SES_NAME}.3 "${OP1}" 'C-m'

tmux attach -t ${SES_NAME}
