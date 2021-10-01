#!/bin/bash

BKUP_DATE=$(date +%Y%m%d_%H%M)
BKUP_DIR="/home/amaze/Desktop/Amaze_Data_${BKUP_DATE}"

# create bkup directory
mkdir -p ${BKUP_DIR}

# sync data
# TODO(scm): TEST THIS!!!
rsync -avP /media/amze/amadee_drive/* ${BKUP_DIR}/
sync

rsync -avP -c /media/amze/amadee_drive/* ${BKUP_DIR}/
sync

rsync -avP -c --remove-source-files /media/amze/amadee_drive/* ${BKUP_DIR}/
sync

echo "Data backup has completed!"

exit 0
