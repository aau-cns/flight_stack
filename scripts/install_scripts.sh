#!/bin/bash

# Copyright (C) 2022 Martin Scheiber,
# Control of Networked Systems, University of Klagenfurt, Austria.
#
# All rights reserved.
#
# This software is licensed under the terms of the BSD-2-Clause-License with
# no commercial use allowed, the full terms of which are made available
# in the LICENSE file. No license in patents is granted.
#
# You can contact the authors at <martin.scheiber@ieee.org>.

override=false

# check if input is -f --> override forcefully
if [ ! -z ${1} ] && [ ${1} == '-f' ]; then
  override=true
# otherwise check if files exist already
elif [[ -f "/usr/bin/fs_op" || -f "/usr/bin/fs_dev1" || -f "/usr/bin/fs_dev2" || -f "/usr/bin/fs_remote" ]]; then
  # they do, so perform user check
  echo "[WARN] files already exist at '/usr/bin/'"
  read -p "       Do you want to override (yN)?" -n 1 -r
  echo
  if [[ $REPLY =~ ^[Yy]$ ]]; then
    override=true
  else
    override=false
  fi
else
  override=true
fi

if [[ "${override}" == true ]]; then
  set -x

  # install the flightstack scripts
  FS_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
  # link scripts, and remove 'scripts' from directory name
  sudo ln -sf ${FS_DIR:0:-8}/src/flightstack/flightstack_scripts/launch_scripts/flightstack_op.sh /usr/bin/fs_op
  sudo ln -sf ${FS_DIR:0:-8}/src/flightstack/flightstack_scripts/launch_scripts/flightstack_dev1.sh /usr/bin/fs_dev1
  sudo ln -sf ${FS_DIR:0:-8}/src/flightstack/flightstack_scripts/launch_scripts/flightstack_dev2.sh /usr/bin/fs_dev2
  sudo ln -sf ${FS_DIR:0:-8}/src/flightstack/flightstack_scripts/launch_scripts/flightstack_remote.sh /usr/bin/fs_remote

  set +x
fi
