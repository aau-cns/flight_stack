#!/bin/bash

px4_topics=(
"/mavros/imu/data_raw"
"/mavros/imu/temperature_imu"
"/mavros/imu/mag"
"/mavros/imu/static_pressure"
"/mavros/global_position/raw/fix"
"/mavros/global_position/raw/gps_vel"
"/mavros/global_position/raw/satellites"
)

ids_camera_topics=( "/mission_cam/image_raw" )

real_sense_topics=(
"/realsense/accel/imu_info"
"/realsense/accel/sample"
"/realsense/gyro/imu_info"
"/realsense/gyro/sample"
"/realsense/odom/sample"
"/realsense/fisheye1/camera_info"
"/realsense/fisheye1/image_raw"
"/realsense/fisheye2/camera_info"
"/realsense/fisheye2/image_raw")

rtk_gps_topic=(
"/rtk_gps_1/fix"
"/rtk_gps_1/fix_velocity"
"/rtk_gps_2/fix"
"/rtk_gps_2/fix_velocity"
)


groups_to_record=(
${px4_topics[@]}
${ids_camera_topics[@]}
${real_sense_topics[@]}
${rtk_gps_topic[@]}
)

topics_to_record=${groups_to_record[@]}

rostopic hz ${topics_to_record}
