#!/bin/bash

# Author: Martin Scheiber

REC_IN="${1}"
PI2_IP=10.42.0.102

CMD_PI1="$(rospack find flightstack_scripts)/record_scripts/record_full_dh.sh"
REM_PI2="ssh -t -t core@${PI2_IP}"
CMD_PI2="$(rospack find flightstack_scripts)/record_scripts/record_full_dh.sh"

PATH_LOCAL="/home/core/rec_local/systems_paper"
PATH_MEDIA="/home/core/rec_media/systems_paper"

if [ "${REC_IN}" == "full" ] || [ "${REC_IN}" == "cam" ] || [ "${REC_IN}" == "sensors" ] ; then
  parallel -u ::: \
  "${CMD_PI1} dev1_${REC_IN} ${PATH_LOCAL} ${PATH_MEDIA}" \
  "${REM_PI2} 'source /home/core/.ros_env.bash; ${CMD_PI2} dev2_${REC_IN} ${PATH_LOCAL} ${PATH_MEDIA}'"

  # ${CMD_PI1} dev1_${REC_IN} ${PATH_LOCAL} ${PATH_MEDIA}
else
  parallel -u ::: "${CMD_PI1} ${REC_IN}" "$CMD_PI2 ${REC_IN}"
  # ${CMD_PI1} ${REC_IN}
fi;
