#!/bin/bash

clear

echo "--------- ----------------------- ----------"
echo "--------- SETUP CAMERA PARAMETERS ----------"
echo "--------- ----------------------- ----------"

cleanup() {

  for pid in "${pids[@]}"; do
    kill -0 "$pid" && kill "$pid"
  done

}

trap cleanup EXIT TERM

# remove old files
rm -f yamlfiles/camera_parameters.yaml

echo "--------- WAIT UNTIL THE CAMERA IS READY ----------"

# launch camera
roslaunch realsense2_camera rs_camera.launch > /dev/null & pids+=( "$!" )

# wait some time to have camera ready
sleep 15

rqt --perspective-file perspective/realsense_cam_params.perspective & pids+=( "$!" )

wait
echo "--------- SETUP CAMERA PARAMETERS DONE ----------"
