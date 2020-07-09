#!/bin/bash

clear

DEVICE_IP=192.168.0.158
USER=core
export ROS_MASTER_URI=http://${DEVICE_IP}:11311

echo "--------- ------------------- ----------"
echo "- START INTRINSIC RECORDING REMOTELY   -"
echo "--------- ------------------- ----------"

# remove old files
cleanup() {


  for pid in "${pids[@]}"; do
    kill -0 "$pid" && kill "$pid"
  done

  #echo "kill rosnode on remote device..."
  #ssh -f ${USER}@${DEVICE_IP} "bash -c 'source ~/.ros_env.bash && rosnode kill -a &'"
  #sleep 5

  # mv intrinsic_recording.bag bagfiles
}

trap cleanup EXIT TERM

echo "--------- LOG IN TO REMOTE DEVICE ----------"
echo ""

# remove old files
ssh -t ${USER}@${DEVICE_IP} "bash -c './catkin_ws/src/amadee/scripts/camera_setup/start_intrinsic_recording_locally.sh'"

echo "--------- COPY BAG FILES ----------"
rsync -azv ${USER}@${DEVICE_IP}:/home/${USER}/bagfiles/intrinsic_recording.bag ./bagfiles/intrinsic_recording.bag


#rosnode kill /intrinsic_bag
echo "--------- LOCAL DONE ----------"
