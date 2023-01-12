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
