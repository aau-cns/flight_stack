#!/bin/bash

#Record
# - two ublox rtk gps position and velocity 
# - ids camera images
# - px4 imu, pressure, magnetometer and GPS

bag_name="outdoor_2_gps"

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

rtk_gps_topic=(
"/rtk_gps_1/fix"
"/rtk_gps_1/fix_velocity"
"/rtk_gps_2/fix"
"/rtk_gps_2/fix_velocity"
)

groups_to_record=(
${px4_topics[@]}
${ids_camera_topics[@]}
${rtk_gps_topic[@]}
)

topics_to_record=${groups_to_record[@]}

echo "Bagname: " ${bag_name}
echo "Topics to record: " ${topics_to_record}

rosbag record --split --size=500 --buffsize=2048 -o ${bag_name} ${topics_to_record}
