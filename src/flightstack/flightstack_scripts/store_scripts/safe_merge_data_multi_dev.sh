#!/bin/bash

# Copyright (C) 2023, Christian Brommer, Martin Scheiber,
# and others, Control of Networked Systems, University of Klagenfurt, Austria.
#
# All rights reserved.
#
# This software is licensed under the terms of the BSD-2-Clause-License with
# no commercial use allowed, the full terms of which are made available
# in the LICENSE file. No license in patents is granted.
#
# You can contact the authors at <christian.brommer@ieee.org>
# and <martin.scheiber@ieee.org>.

# This script assumes two devices with two folders each. In this particular case
# one folder is for local recodring on a device and the second folder
# is for recording on a media device.
# This script merges the two folder from a remote device and the local folder on
# the current host, to the media folder of the current host

remote_ssh='core@10.42.0.102'

rmt_local_src_dir=${remote_ssh}':/home/core/rec_local/'
rmt_media_src_dir=${remote_ssh}':/home/core/rec_media/'

host_local_src_dir='/home/core/rec_local/'
host_media_src_dir='/home/core/rec_media/'

dest_folder='/final/'
dest_dir='/home/core/rec_media/'$dest_folder

host_local_dest_pref='/dev1/sensors/'
host_media_dest_pref='/dev1/cam/'

rmt_local_dest_pref='/dev2/sensors/'
rmt_media_dest_pref='/dev2/cam/'


t_string=$(date +%Y%d%m%H%M%S)

sync_dir_1=($rmt_media_src_dir $dest_dir$t_string$rmt_media_dest_pref)
sync_dir_2=($rmt_local_src_dir $dest_dir$t_string$rmt_local_dest_pref)
sync_dir_3=($host_media_src_dir $dest_dir$t_string$host_media_dest_pref)
sync_dir_4=($host_local_src_dir $dest_dir$t_string$host_local_dest_pref)

sync_vector=(
sync_dir_1[@]
sync_dir_2[@]
sync_dir_3[@]
sync_dir_4[@]
)

len=${#sync_vector[@]}

for ((k=0; k<$len; k++)); do
src_dir=${!sync_vector[k]:0:1}
dest_dir=${!sync_vector[k]:1:1}
echo $src_dir to $dest_dir

mkdir -p $dest_dir

# Ensure all data is written before copy, local and remote
sync || ssh $remote_ssh sync

# Copy data
rsync -ravPh --exclude=$dest_folder $src_dir $dest_dir
sync || ssh $remote_ssh sync

# Copy data and ensure the checksum is correct
rsync -ravPh -c --exclude=$dest_folder $src_dir $dest_dir
sync || ssh $remote_ssh sync

# Copy data and delete the source
rsync -ravPh -c --remove-source-files --exclude=$dest_folder $src_dir $dest_dir
sync || ssh $remote_ssh sync

done
