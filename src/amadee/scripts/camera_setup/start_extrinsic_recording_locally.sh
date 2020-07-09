#!/bin/bash

clear

# Run a command in the background.
_evalBg() {
    eval "$@" &>/dev/null & disown;
}

echo "--------- ------------------- ----------"
echo "--------- EXTRINSIC RECORDING ----------"
echo "--------- ------------------- ----------"

# killing
cleanup() {
  for pid in "${pids[@]}"; do
    kill -0 "$pid" && kill "$pid"
  done
  sleep 5
  mv extrinsic_recording.bag bagfiles/
}

trap cleanup EXIT TERM

# remove old files
rm -f extrinsic_recording.bag.active
rm -f extrinsic_recording.bag
#rm -f bagfiles/extrinsic_recording.bag

echo "--------- WAIT UNTIL THE CAMERA IS READY ----------"
source ~/.ros_env.bash

cmd="roslaunch amadee_bringup ueye_cam_no_param.launch";
_evalBg "${cmd}";
sleep 5

# throttle messages to correct framerate
rosparam set /ueye_cam_nodelet/frame_rate 20.0

# record .bag file
rosbag record /camera/image_raw -O extrinsic_recording

echo "--------- EXTRINSIC RECORDING DONE ----------"
