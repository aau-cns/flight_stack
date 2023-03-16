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
REC_LOCAL="${HOME}/recordings"    # default local path of record_full.sh
REC_MEDIA="${HOME}/recordings"    # default media path of record_full.sh
REC_LOGS="${HOME}/.ros/log"       # default logging path of autonomy/ros

DEST_DIR="${REC_MEDIA}/final" # default destination path (media device)

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
while getopts vhzd:l:m:r: flag
do
  case "${flag}" in
    l) REC_LOCAL=${OPTARG};;
    m) REC_MEDIA=${OPTARG};;
    r) REC_LOGS=${OPTARG};;
    d) DEST_DIR=${OPTARG};;

    v) B_DEBUG_ON=true;;
    z) B_CREATE_TARBALL=true;;
    h) print_help;;

    *) echo "Unknown option ${flag}"; print_help;;
  esac
done
shift $((OPTIND-1))


# check for local/media paths
if [ -z "${REC_LOCAL}" ]; then
  echo -e "${COL_WARN}No local path provided, recording to home directory: '${HOME}/recordings'${NC}"
  REC_LOCAL="${HOME}/recordings"
fi
if [ -z "${REC_MEDIA}" ]; then
  echo -e "${COL_WARN}No media path provided, recording to local path: '${REC_MEDIA}'.${NC}"
  REC_MEDIA="${REC_LOCAL}"
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

# setup rsync basic command
RSYNC_CMD="rsync -rptgoDL"

if [ ${B_DEBUG_ON} = true ]; then
  set -x
  # update rsync cmd to include verbose output
  RSYNC_CMD="${RSYNC_CMD} -vPh"
fi

# setup directory name (timestamp)
TIMESTAMP=$(date +%Y%m%d-%H%M%S)
DIR_NAME="${DEST_DIR}/${TIMESTAMP}"

# Ensure all data is written before copy
sync


if [ ${B_CREATE_TARBALL} = true ]; then
  # create folder structure with symlinks (to then compress)
  tmp_dir=/tmp/recordings/${TIMESTAMP}
  rm -rf ${tmp_dir}
  mkdir -p ${tmp_dir}/logs/ros
  mkdir -p ${tmp_dir}/logs/autonomy
  
  # link the files to the corresponding directories
  ln -s ${REC_LOCAL}/* ${tmp_dir}/
  rm -rf ${tmp_dir}/final
  if [ "${REC_LOCAL}" != "${REC_MEDIA}" ]; then
    # only link rec_media if it is not the same as rec_local
    ln -s ${REC_MEDIA}/* ${tmp_dir}/
    rm -rf ${tmp_dir}/final
  fi
  ln -s ${REC_LOGS}/autonomy/*  ${tmp_dir}/logs/autonomy/
  ln -s ${REC_LOGS}/latest/*  ${tmp_dir}/logs/ros/

  # create tarball from tmp dir
  tar --dereference -czvf ${DIR_NAME}.tar.gz -C ${tmp_dir}/.. ${TIMESTAMP}
  
  # ensure all data is written before deletions
  sync
  
  # store autonomy logfile with ROS logs for later usage
  ${RSYNC_CMD} ${REC_LOGS}/autonomy/ ${REC_LOGS}/latest/

  # remove source files
  rm -rf $(find ${REC_LOCAL} -type f -not -path "*/final/*") $(find ${REC_MEDIA} -type f -not -path "*/final/*") ${REC_LOGS}/autonomy/
  rm -rf ${tmp_dir}
else
  # create folder structure
  mkdir -p ${DIR_NAME}/logs/autonomy ${DIR_NAME}/logs/ros

  # Copy data
  ${RSYNC_CMD} ${REC_LOCAL}/ ${REC_MEDIA}/ ${DIR_NAME} \
    --exclude='final/'
  ${RSYNC_CMD} ${REC_LOGS}/autonomy/ ${DIR_NAME}/logs/autonomy/
  ${RSYNC_CMD} ${REC_LOGS}/latest/ ${DIR_NAME}/logs/ros/
  sync

  # Copy data and ensure the checksum is correct
  ${RSYNC_CMD} -c ${REC_LOCAL}/ ${REC_MEDIA}/ ${DIR_NAME} \
    --exclude='final/'
  ${RSYNC_CMD} -c ${REC_LOGS}/autonomy/ ${DIR_NAME}/logs/autonomy/
  ${RSYNC_CMD} -c ${REC_LOGS}/latest/ ${DIR_NAME}/logs/ros/
  # sync

  # Store autonomy logfile with ROS logs for later usage
  ${RSYNC_CMD} ${REC_LOGS}/autonomy/ ${REC_LOGS}/latest/

  # Copy data and delete the source
  ${RSYNC_CMD} -c --remove-source-files ${REC_LOCAL}/ ${REC_MEDIA}/ ${DIR_NAME} \
    --exclude='final/'
  ${RSYNC_CMD} -c --remove-source-files ${REC_LOGS}/autonomy/ ${DIR_NAME}/logs/autonomy/
  # ${RSYNC_CMD} -c --remove-source-files ${REC_LOGS}/latest/ ${DIR_NAME}/logs/ros
  sync
fi

if [ ${B_DEBUG_ON} = true ]; then
  set +x
fi
