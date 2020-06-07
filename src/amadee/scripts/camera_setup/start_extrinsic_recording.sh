#!/bin/bash

clear

echo "--------- ------------------- ----------"
echo "--------- EXTRINSIC RECORDING ----------"
echo "--------- ------------------- ----------"

# killing
cleanup() {
  for pid in "${pids[@]}"; do
    kill -0 "$pid" && kill "$pid"
  done
  sleep 0.5
  mv extrinsic_recording.bag bagfiles
}

trap cleanup EXIT TERM

# remove old files
rm -f extrinsic_recording.bag.active
rm -f extrinsic_recording.bag
rm -f bagfiles/extrinsic_recording.bag

echo "--------- WAIT UNTIL THE CAMERA IS READY ----------"

# launch camera
roslaunch realsense2_camera rs_camera.launch > /dev/null & pids+=( "$!" )

# wait some time to have camera ready
sleep 15

# load saved parameters

# throttle messages to correct framerate
rosrun topic_tools throttle messages /camera/color/image_raw 20.0 throttle_camera_20hz & pids+=( "$!" )

# record .bag file
rosbag record throttle_camera_20hz --publish --output-name=extrinsic_recording __name:=extrinsic_bag &

# visualize camera stream while recording
rqt --perspective-file perspective/realsense_recording.perspective & pids+=( "$!" )

wait
#rosnode kill /extrinsic_bag
echo "--------- EXTRINSIC RECORDING DONE ----------"

