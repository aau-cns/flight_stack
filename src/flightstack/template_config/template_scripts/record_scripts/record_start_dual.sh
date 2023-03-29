#!/bin/bash

# Copyright (C) 2023, Martin Scheiber, Christian Brommer
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
DEV2_IP=10.42.0.102

CMD_DEV1="$(rospack find flightstack_scripts)/record_scripts/record_full.sh"
REM_DEV2="ssh -t -t core@${DEV2_IP}"
CMD_DEV2="$(rospack find flightstack_scripts)/record_scripts/record_full.sh"

DIR_NAME="flightstack"
PATH_LOCAL="/home/core/rec_local/${DIR_NAME}"
PATH_MEDIA="/home/core/rec_media/${DIR_NAME}"

if [ "${REC_IN}" == "full" ] || [ "${REC_IN}" == "cam" ] || [ "${REC_IN}" == "sensors" ] ; then
  parallel -u ::: \
  "${CMD_DEV1} dev1_${REC_IN} -l ${PATH_LOCAL} -m ${PATH_MEDIA}" \
  "${REM_DEV2} 'source /home/core/.ros_env.bash; ${CMD_DEV2} dev2_${REC_IN} -l ${PATH_LOCAL} -m ${PATH_MEDIA}'"

  # ${CMD_DEV1} dev1_${REC_IN} ${PATH_LOCAL} ${PATH_MEDIA}
else
  parallel -u ::: "${CMD_DEV1} ${REC_IN}" "$CMD_DEV2 ${REC_IN}"
  # ${CMD_DEV1} ${REC_IN}
fi;
