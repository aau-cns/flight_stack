#!/bin/bash

bag_name="outdoor"

px4_topics=(
"/mavros/imu/data_raw"
"/mavros/imu/temperature_imu"
"/mavros/imu/mag"
"/mavros/imu/static_pressure"
"/mavros/global_position/raw/fix"
"/mavros/global_position/raw/gps_vel"
"/mavros/global_position/raw/satellites"
)

ids_camera_topics=( "/camera/image_raw" )
real_sense_topics=(
"/camera/accel/imu_info"
"/camera/accel/sample"
"/camera/gyro/imu_info"
"/camera/gyro/sample"
"/camera/odom/sample"
"/camera/fisheye1/camera_info"
"/camera/fisheye1/image_raw"
"/camera/fisheye2/camera_info"
"/camera/fisheye2/image_raw")

rtk_gps_topic=(
"/rtk/fix"
"/rtk/fix_velocity"
)

groups_to_record=(
${px4_topics[@]}
${ids_camera_topics[@]}
${real_sense_topics[@]}
${rtk_gps_topic[@]}
)

topics_to_record=${groups_to_record[@]}

# for i in "${px4_topics[@]}" "${ids_camera_topics[@]}" "${real_sense_topics[@]}" "${rtk_gps_topic[@]}"; do
# 	topics_to_record+=" $i";
# done

echo "Bagname: " ${bag_name}
echo "Topics to record: " ${topics_to_record}

rosbag record --split --size=500 --buffsize=2048 -o ${bag_name} ${topics_to_record}
