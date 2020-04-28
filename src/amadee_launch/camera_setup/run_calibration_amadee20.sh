#!/bin/bash

clear

echo "--------- -------------------- ----------"
echo "--------- CALIBRATION AMADEE20 ----------"
echo "--------- -------------------- ----------"

echo "--------- ITRINSIC PARAMETER CALIBRATION ----------"

kalibr_calibrate_cameras --bag bagfiles/intrinsic_recording.bag --topics throttle_camera_4hz --models omni-radtan --target yamlfiles/target.yaml

echo "--------- ITRINSIC PARAMETER CALIBRATION ----------"

kalibr_calibrate_imu_camera --bag bagfiles/extrinsic_recording.bag --cam yamlfiles/camchain.yaml --imu yamlfiles/imu.yaml --target yamlfiles/target.yaml

echo "--------- CALIBRATION AMADEE20 DONE ----------"
