#!/bin/bash

# Copyright (C) 2022, Martin Scheiber, Christian brommer
# and others, Control of Networked Systems, University of Klagenfurt, Austria.
#
# All rights reserved.
#
# This software is licensed under the terms of the BSD-2-Clause-License with
# no commercial use allowed, the full terms of which are made available
# in the LICENSE file. No license in patents is granted.
#
# You can contact the authors at <martin.scheiber@ieee.org>
# and <christian.brommer@ieee.org>.

REC_IN="${1}"
PI2_IP=10.42.0.102

CMD_PI1="$(rospack find flightstack_scripts)/record_scripts/record_full_dh.sh"
REM_PI2="ssh -t -t core@${PI2_IP}"
CMD_PI2="$(rospack find flightstack_scripts)/record_scripts/record_full_dh.sh"

DIR_NAME="flightstack"
PATH_LOCAL="/home/core/rec_local/${DIR_NAME}"
PATH_MEDIA="/home/core/rec_media/${DIR_NAME}"

if [ "${REC_IN}" == "full" ] || [ "${REC_IN}" == "cam" ] || [ "${REC_IN}" == "sensors" ] ; then
  parallel -u ::: \
  "${CMD_PI1} dev1_${REC_IN} ${PATH_LOCAL} ${PATH_MEDIA}" \
  "${REM_PI2} 'source /home/core/.ros_env.bash; ${CMD_PI2} dev2_${REC_IN} ${PATH_LOCAL} ${PATH_MEDIA}'"

  # ${CMD_PI1} dev1_${REC_IN} ${PATH_LOCAL} ${PATH_MEDIA}
else
  parallel -u ::: "${CMD_PI1} ${REC_IN}" "$CMD_PI2 ${REC_IN}"
  # ${CMD_PI1} ${REC_IN}
fi;
