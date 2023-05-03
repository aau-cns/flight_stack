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

################################################################################
# Global Variables                                                             #
################################################################################

# setup colors
COL_ERR='\033[0;31m'  #Red Color
COL_WARN='\033[0;33m' #Yellow Color
NC='\033[0m'          #No Color

# SCRIPT VARIABLES
# REC_LOCAL="${HOME}/recordings"    # default local path of record_full.sh
# REC_MEDIA="${HOME}/recordings"    # default media path of record_full.sh
# REC_LOGS="${HOME}/.ros/log"       # default logging path of autonomy/ros

# DEST_DIR="${REC_MEDIA}/final" # default destination path (media device)

B_FAST_SYNC=false
B_CREATE_TARBALL=false
B_DEBUG_ON=false

################################################################################
# Help                                                                         #
################################################################################
print_help(){
    echo "USAGE: ${script_name} [OPTIONS]"
    echo ""
    echo "  Options:"
    echo "    -l PATH       path to internal recordings media"
    echo "    -m PATH       path to external recordings media device (for images)"
    echo "    -r PATH       path to ROS logs (default \${HOME}/.ros/log)"
    echo "    -d NAME       path to destination storage (default: /<MEDIA_PATH>/final)"
    echo ""
    echo "    -f            fast syncing (skips checksum test and immediatly deletes files)"
    echo "    -z            create tarball of all files in dest dir"
    echo "                  instead of copying raw files"
    echo "    -v            enable detailed debug output"
    echo ""
    echo "    -h        print this help"
    echo ""
    exit 0;
}

################################################################################
# Execution Options                                                            #
################################################################################

# parse flags
while getopts vhfzd:l:m:r: flag
do
  case "${flag}" in
    l) REC_LOCAL=${OPTARG};;
    m) REC_MEDIA=${OPTARG};;
    r) REC_LOGS=${OPTARG};;
    d) DEST_DIR=${OPTARG};;

    f) B_FAST_SYNC=true;;
    v) B_DEBUG_ON=true;;
    z) B_CREATE_TARBALL=true;;
    h) print_help;;

    *) echo "Unknown option ${flag}"; print_help;;
  esac
done
shift $((OPTIND-1))


# check for local/media paths
if [ -z "${REC_LOCAL}" ]; then
  REC_LOCAL="${HOME}/recordings"
  echo -e "${COL_WARN}No local path provided, using home directory: '${REC_LOCAL}'${NC}"
fi
if [ -z "${REC_MEDIA}" ]; then
  REC_MEDIA="${REC_LOCAL}"
  echo -e "${COL_WARN}No media path provided, using local path: '${REC_MEDIA}'.${NC}"
fi
if [ -z "${REC_LOGS}" ]; then
  REC_LOGS="${HOME}/.ros/log"
  echo -e "${COL_WARN}No ros log path provided, using home directory: '${REC_LOGS}'.${NC}"
fi
if [ -z "${DEST_DIR}" ]; then
  DEST_DIR="${REC_MEDIA}/final"
  echo -e "${COL_WARN}No destination path provided, using media path: '${DEST_DIR}'.${NC}"
fi


# check if local/media directories exist
if [ ! -d "${REC_LOCAL}" ]; then
  echo -e "${COL_ERR}[ERROR] ${REC_LOCAL} does not exist ${NC}"
  exit 1;
fi
if [ ! -d "${REC_MEDIA}" ]; then
  echo -e "${COL_ERR}[ERROR] ${REC_MEDIA} does not exist ${NC}"
  exit 1;
fi
if [ ! -d "${REC_LOGS}" ]; then
  echo -e "${COL_ERR}[ERROR] ${REC_LOGS} does not exist ${NC}"
  exit 1;
fi

# check if destination directories exist
if [ ! -d "${DEST_DIR}" ]; then
  echo -e "${COL_WARN}${DEST_DIR} does not exist, creating it... ${NC}"
  mkdir -p ${DEST_DIR}
fi

################################################################################
################################################################################
# MAIN SCRIPT                                                                  #
################################################################################
################################################################################

if [ ${B_DEBUG_ON} = true ]; then
  set -x
fi

# setup directory name (timestamp)
TIMESTAMP=$(date +%Y%m%d-%H%M%S)
DIR_NAME="${DEST_DIR}/${TIMESTAMP}"

# setup rsync basic command
RSYNC_CMD="rsync -rptgoDL"

# check if rsync is permited with these options to destdir
echo "testing rsync" >> /tmp/rsync_test.log 
${RSYNC_CMD} /tmp/rsync_test.log ${DIR_NAME}/
if [ $? -eq 0 ]; then
  # rsync succeeded, remove test file
  rm -rf ${DIR_NAME}/rsync_test.log
else
  # failed rsync, change to simpler version, fewer permitions
  RSYNC_CMD="rsync -rpDL -A --no-perms"

  # remove test file, if exists
  rm -rf ${DIR_NAME}/rsync_test.log
fi

if [ ${B_DEBUG_ON} = true ]; then
  # update rsync cmd to include verbose output
  RSYNC_CMD="${RSYNC_CMD} -vPh"
fi

# Ensure all data is written before copy
sync


if [ ${B_CREATE_TARBALL} = true ]; then
  # create folder structure with symlinks (to then compress)
  tmp_dir=/tmp/recordings/${TIMESTAMP}
  rm -rf ${tmp_dir}
  mkdir -p ${tmp_dir}/logs/ros
  mkdir -p ${tmp_dir}/logs/autonomy
  
  # link the files to the corresponding directories, if they are not empty
  if [ -n "$(ls -A ${REC_LOCAL} 2> /dev/null)" ]; then
    ln -s ${REC_LOCAL}/* ${tmp_dir}/
    rm -rf ${tmp_dir}/final
  fi
  if [ "${REC_LOCAL}" != "${REC_MEDIA}" ]; then
    # only link rec_media if it is not the same as rec_local
    if [ -n "$(ls -A ${REC_MEDIA} 2> /dev/null)" ]; then
      ln -s ${REC_MEDIA}/* ${tmp_dir}/
      rm -rf ${tmp_dir}/final
    fi
  fi
  if [ -n "$(ls -A ${REC_LOGS}/autonomy 2> /dev/null)" ]; then
    ln -s ${REC_LOGS}/autonomy/*  ${tmp_dir}/logs/autonomy/
  fi
  if [ -n "$(ls -A ${REC_LOGS}/latest 2> /dev/null)" ]; then
    ln -s ${REC_LOGS}/latest/*  ${tmp_dir}/logs/ros/
  fi

  # create tarball from tmp dir
  tar --dereference -czvf ${DIR_NAME}.tar.gz -C ${tmp_dir}/.. ${TIMESTAMP}
  
  # ensure all data is written before deletions
  sync
  
  # store autonomy logfile with ROS logs for later usage
  if [ -n "$(ls -A ${REC_LOGS}/autonomy 2> /dev/null)" ]; then
    ${RSYNC_CMD} ${REC_LOGS}/autonomy/ ${REC_LOGS}/latest/
  fi

  # remove source files
  rm -rf $(find ${REC_LOCAL} -type f -not -path "*/final/*") $(find ${REC_MEDIA} -type f -not -path "*/final/*") ${REC_LOGS}/autonomy/
  rm -rf ${tmp_dir}
else
  # create folder structure
  mkdir -p ${DIR_NAME}/logs/autonomy ${DIR_NAME}/logs/ros

  # Store autonomy logfile with ROS logs for later usage
  ${RSYNC_CMD} ${REC_LOGS}/autonomy/ ${REC_LOGS}/latest/

  if [ ${B_FAST_SYNC} != true ]; then
    # Copy data
    ${RSYNC_CMD} ${REC_LOCAL}/ ${REC_MEDIA}/ ${DIR_NAME} \
      --exclude='final/'
    ${RSYNC_CMD} ${REC_LOGS}/autonomy/ ${DIR_NAME}/logs/autonomy/
    ${RSYNC_CMD} ${REC_LOGS}/latest/ ${DIR_NAME}/logs/ros/
    sync

    # add rsync -c option
    RSYNC_CMD="${RSYNC_CMD} -c"

    # Copy data and ensure the checksum is correct
    ${RSYNC_CMD} ${REC_LOCAL}/ ${REC_MEDIA}/ ${DIR_NAME} \
      --exclude='final/'
    ${RSYNC_CMD} ${REC_LOGS}/autonomy/ ${DIR_NAME}/logs/autonomy/
    ${RSYNC_CMD} ${REC_LOGS}/latest/ ${DIR_NAME}/logs/ros/
    # sync
  fi    
  
  # Copy data and delete the source
  ${RSYNC_CMD} --remove-source-files ${REC_LOCAL}/ ${REC_MEDIA}/ ${DIR_NAME} \
    --exclude='final/'
  ${RSYNC_CMD} --remove-source-files ${REC_LOGS}/autonomy/ ${DIR_NAME}/logs/autonomy/
  # ${RSYNC_CMD} -c --remove-source-files ${REC_LOGS}/latest/ ${DIR_NAME}/logs/ros
  sync
fi

if [ ${B_DEBUG_ON} = true ]; then
  set +x
fi

echo -e "    --> data stored in: ${DIR_NAME}"
