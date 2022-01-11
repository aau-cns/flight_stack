#!/bin/bash

# Optimal settings to record all RealSense data on a Raspberry PI4 without triggering an internal overflow
# tcpnodelay: Do not wait for ACK after each small package before sending the next package

rosbag record --tcpnodelay -b 0 --split --size=1000 /realsense/accel/imu_info /realsense/accel/sample /realsense/fisheye1/camera_info /realsense/fisheye1/image_raw /realsense/fisheye2/camera_info /realsense/fisheye2/image_raw /realsense/gyro/ imu_info /realsense/gyro/sample /realsense/odom/sample
