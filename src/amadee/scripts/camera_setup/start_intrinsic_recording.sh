#!/bin/bash

clear

echo "--------- ------------------- ----------"
echo "--------- INTRINSIC RECORDING ----------"
echo "--------- ------------------- ----------"

# remove old files
cleanup() {
  for pid in "${pids[@]}"; do
    kill -0 "$pid" && kill "$pid"
  done
  sleep 0.5
  mv intrinsic_recording.bag bagfiles
}

trap cleanup EXIT TERM

# remove old files
rm -f intrinsic_recording.bag.active
rm -f intrinsic_recording.bag
rm -f bagfiles/intrinsic_recording.bag

# load saved parameters

echo "--------- WAIT UNTIL THE CAMERA IS READY ----------"

# launch camera
roslaunch realsense2_camera rs_camera.launch > /dev/null & pids+=( "$!" )

# wait some time to have camera ready
sleep 15

# throttle messages to correct framerate
rosrun topic_tools throttle messages /camera/color/image_raw 4.0 throttle_camera_4hz & pids+=( "$!" )

# record .bag file
rosbag record throttle_camera_4hz --publish --output-name=intrinsic_recording __name:=intrinsic_bag &

# visualize camera stream while recording
rqt --perspective-file perspective/realsense_recording.perspective & pids+=( "$!" )

wait
#rosnode kill /intrinsic_bag
echo "--------- INTRINSIC RECORDING DONE ----------"
