#!/bin/bash

bag_name="all_gps"

px4_topics=(
"/mavros/global_position/raw/fix"
"/mavros/global_position/raw/gps_vel"
"/mavros/global_position/raw/satellites"
)

rtk_gps1_topic=(
"/rtk_gps_1/aidalm"
"/rtk_gps_1/aideph"
"/rtk_gps_1/fix"
"/rtk_gps_1/fix_velocity"
"/rtk_gps_1/monhw"
"/rtk_gps_1/navclock"
"/rtk_gps_1/navpvt"
"/rtk_gps_1/navsat"
"/rtk_gps_1/navstatus"
"/rtk_gps_1/navsvin"
)

rtk_gps2_topic=(
"/rtk_gps_2/aidalm"
"/rtk_gps_2/aideph"
"/rtk_gps_2/fix"
"/rtk_gps_2/fix_velocity"
"/rtk_gps_2/monhw"
"/rtk_gps_2/navclock"
"/rtk_gps_2/navpvt"
"/rtk_gps_2/navsat"
"/rtk_gps_2/navstatus"
"/rtk_gps_2/navsvin"
)

groups_to_record=(
${px4_topics[@]}
${rtk_gps1_topic[@]}
${rtk_gps2_topic[@]}
)

topics_to_record=${groups_to_record[@]}

echo "Bagname: " ${bag_name}
echo "Topics to record: " ${topics_to_record}

rosbag record --split --size=500 -o ${bag_name} ${topics_to_record}
