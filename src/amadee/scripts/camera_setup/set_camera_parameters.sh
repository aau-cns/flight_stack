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

DEVICE_IP=192.168.0.158
USER=core
export ROS_MASTER_URI=http://${DEVICE_IP}:11311

trap cleanup EXIT TERM

# remove old files
rm -f yamlfiles/camera_parameters.yaml

echo "--------- WAIT UNTIL THE CAMERA IS READY ----------"
echo ""

# launch camera
ssh -f ${USER}@${DEVICE_IP} "bash -c 'source ~/.ros_env.bash && roscore >/dev/null 2>&1 &'"
until rostopic list >/dev/null 2>&1; do sleep 1; done
ssh -f ${USER}@${DEVICE_IP} "bash -c 'source ~/.ros_env.bash && roslaunch amadee_bringup ueye_cam.launch >/dev/null 2>&1 &'"

# wait some time to have camera ready
sleep 10

echo "------------ OPENING CAMERA SETUP GUI -------------"
echo ""

rqt --perspective-file perspective/ids_cam_params.perspective & pids+=( "$!" )

echo "--------------- PRESS 's' TO SAVE -----------------"
echo ""

read -n 1 k <&1

echo ""

if [[ $k = s ]] ; then

#echo "[TODO] dumping parameters..."

#rosrun dynamic_reconfigure dynparam dump /camera yamlfiles/camera_parameters.yaml

#printf "\n---------------- PARAMETER SAVED ------------------\n\n"
fi

echo "-------------- PRESS [CTRL-C] TO EXIT ---------------"
echo ""

wait
