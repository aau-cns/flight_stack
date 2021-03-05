#!/bin/bash

src_dir='/home/core/rec_local/'
dest_dir='/home/core/rec_media/'

# Ensure all data is written before copy
sync

# Copy data
rsync -ravPh $src_dir $dest_dir
sync
# Copy data and ensure the checksum is correct
rsync -ravPh -c $src_dir $dest_dir
sync
# Copy data and delete the source
rsync -ravPh -c --remove-source-files $src_dir $dest_dir
sync

# Delete empty dir in source, mindepth prevents the deletion of the sourcefolder itself
find $src_dir -mindepth 1 -type d -empty -delete
