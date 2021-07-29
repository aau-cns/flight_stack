#!/bin/bash

# Author: Martin Scheiber

REC_IN="${1}"
PI2_IP=10.42.0.102

# CMD_PI1="/home/core/catkin_ws/src/amadee/scripts/record_scripts/record_full_setup.sh"
# CMD_PI1="ssh core@${PI2_IP}:/home/core/catkin_ws/src/amadee/scripts/record_scripts/record_full_setup.sh"
# CMD_PI1="/data/WS/ros_ws/aaucns_ws/amadee20_cws/src/amadee/scripts/record_scripts/dummy/sleep_dummy.sh"
# CMD_PI2="/data/WS/ros_ws/aaucns_ws/amadee20_cws/src/amadee/scripts/record_scripts/dummy/sleep_dummy.sh"
CMD_PI1="/data/dummy/sleep_dummy.sh"
CMD_PI2="/data/dummy/sleep_dummy.sh"

if [ "${REC_IN}" == "full" ] || [ "${REC_IN}" == "cam" ] || [ "${REC_IN}" == "cam" ] ; then
  parallel -u --citation ::: "${CMD_PI1} dev1_${REC_IN}" "$CMD_PI2 dev2_${REC_IN}"
else
  parallel -u --citation ::: "${CMD_PI1} ${REC_IN}" "$CMD_PI2 ${REC_IN}"
fi;
