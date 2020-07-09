#!/bin/bash

clear

# Run a command in the background.
_evalBg() {
    eval "$@" &>/dev/null & disown;
}

echo "--------- ------------------- ----------"
echo "--------- INTRINSIC RECORDING ----------"
echo "--------- ------------------- ----------"

# remove old files
cleanup() {
  for pid in "${pids[@]}"; do
    kill -0 "$pid" && kill "$pid"
  done
  sleep 5
  mv intrinsic_recording.bag bagfiles/
}
trap cleanup EXIT TERM

# remove old files
rm -f intrinsic_recording.bag.active
rm -f intrinsic_recording.bag
rm -f bagfiles/intrinsic_recording.bag

# load saved parameters

echo "--------- WAIT UNTIL THE CAMERA IS READY ----------"
source ~/.ros_env.bash

cmd="roslaunch amadee_bringup ueye_cam.launch";
_evalBg "${cmd}";
sleep 5
# launch camera
# wait some time to have camera ready


# throttle messages to correct framerate
rosparam set /ueye_cam_nodelet/frame_rate 4.0

# record .bag file
rosbag record /camera/image_raw -O intrinsic_recording
# _evalBg "${cmd}";
# visualize camera stream while recording
#rqt --perspective-file perspective/realsense_recording.perspective & pids+=( "$!" )

#rosnode kill /intrinsic_bag
echo "--------- INTRINSIC RECORDING DONE ----------"
