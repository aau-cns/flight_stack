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

# This script syncs multiple devices to the destination directory

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

SSH_REMOTES=(
"localhost",
"core@10.42.0.102"
)

NUM_DEVICES=2

B_CREATE_TARBALL=false
B_DEBUG_ON=false

################################################################################
# Help                                                                         #
################################################################################
print_help(){
    echo "USAGE: ${script_name} <NUM> [OPTIONS]"
    echo ""
    echo "  Arguments:"
    echo "    NUM           number of devices to sync"
    echo ""
    echo "  Options:"
    echo "    -i REMOTES    ssh login credentials"
    echo "                  use 'localhost' for current device"
    echo "    -l ID PATH    paths to internal recordings media"
    echo "    -m ID PATH    paths to external recordings media device (for images)"
    echo "    -r ID PATH    paths to ROS logs (default \${HOME}/.ros/log)"
    echo "    -d NAME       paths to destination storage (default: /data/recordings/final)"
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

# parse NUM
if [ -z ${1} ]; then
  echo -e "${COL_ERR}[ERROR] number of devices missing${NC}"
  print_help;
  exit 1;
elif [[ ${1} =~ '^[0-9]+$' ]]; then
  echo -e "${COL_ERR}[ERROR] '${1}' is not an integer number${NC}"
  print_help;
  exit 1;
else
  NUM_DEVICES=${1}
fi
# shift optind to next value
OPTIND=$((OPTIND+1));

# parse flags
while getopts :hzvi:d:l::m:r: flag
do
  case "${flag}" in
    i) 
      SSH_REMOTES=()
      idx=$((OPTIND-1))
      for ((n=0; n<${NUM_DEVICES}; n++)); do
        SSH_REMOTES+=("${!idx}");
        idx=$((idx+1));
      done;
      OPTIND=$((idx));;
    l) 
      declare REC_LOCAL_${OPTARG}=${!OPTIND};
      OPTIND=$((OPTIND+1));;
    m)
      declare REC_MEDIA_${OPTARG}=${!OPTIND};
      OPTIND=$((OPTIND+1));;
    r)
      declare REC_LOGS_${OPTARG}=${!OPTIND};
      OPTIND=$((OPTIND+1));;
    d) DEST_DIR=${OPTARG};;

    v) B_DEBUG_ON=true;;
    z) B_CREATE_TARBALL=true;;
    h) print_help;;

    *) echo "Unknown option ${flag}"; print_help;;
  esac
done
shift "$((OPTIND-1))"

# check if destination directories exist
if [ ! -d "${DEST_DIR}" ]; then
  echo -e "${COL_WARN}${DEST_DIR} does not exist, creating it... ${NC}"
  mkdir -p ${DEST_DIR}
fi

# debug output
if [ ${B_DEBUG_ON} = true ]; then
  echo "NUM_DEVICES:    ${NUM_DEVICES}"
  echo "REMOTE_SSH[0]:  ${SSH_REMOTES[0]}"
  echo "REC_LOCAL_0:    ${REC_LOCAL_0}"
  echo "REC_MEDIA_0:    ${REC_MEDIA_0}"
  echo "REC_LOGS_0:     ${REC_LOGS_0}"
  echo "REMOTE_SSH[1]:  ${SSH_REMOTES[1]}"
  echo "REC_LOCAL_1:    ${REC_LOCAL_1}"
  echo "REC_MEDIA_1:    ${REC_MEDIA_1}"
  echo "REC_LOGS_1:     ${REC_LOGS_1}"
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

# create folder structure with symlinks and files(to then compress)
# in case tarball is used
TAR_TMP_DIR=/tmp/recordings/${TIMESTAMP}
rm -rf ${TAR_TMP_DIR}
mkdir -p ${TAR_TMP_DIR}/logs/ros
mkdir -p ${TAR_TMP_DIR}/logs/autonomy

RSYNC_CMD="rsync -rptgoDvPhL"
RM_CMD="rm -rf ${TAR_TMP_DIR}"
for ((n=0; n<${NUM_DEVICES}; n++)); do
  # get ssh command
  remote_ssh=${SSH_REMOTES[n]}
  sync_cmd="ssh ${remote_ssh} 'sync'"
  on_localhost=false
  if [ ${remote_ssh} = localhost ]; then
    remote_ssh=""
    sync_cmd="sync"
    on_localhost=true
  else
    remote_ssh="${remote_ssh}:"
  fi

  # set source directories
  var_local="REC_LOCAL_${n}"
  var_media="REC_MEDIA_${n}"
  var_logs="REC_LOGS_${n}"

  # check if variables are set, otherwise use default
  if [ -z "${!var_local}" ]; then
    echo "${COL_WARN}No local path provided for dev ${n}, using default directory: '${REC_LOCAL}'${NC}"
    declare ${!var_local}=${REC_LOCAL}
  fi
  if [ -z "${!var_media}" ]; then
    echo "${COL_WARN}No media path provided for dev ${n}, using default directory: '${REC_MEDIA}'${NC}"
    declare ${!var_media}=${REC_MEDIA}
  fi
  if [ -z "${!var_logs}" ]; then
    echo "${COL_WARN}No logs path provided for dev ${n}, using default directory: '${REC_LOGS}'${NC}"
    declare ${!var_logs}=${REC_LOGS}
  fi

  # set source destinations
  local_src_dir="${remote_ssh}${!var_local}"
  media_src_dir="${remote_ssh}${!var_media}"
  logs_src_dir="${remote_ssh}${!var_logs}"

  # check if dirrectories exits
  # if [ ! -d "${local_src_dir}" ]; then
  #   echo "${COL_ERR}[ERROR] ${local_src_dir} does not exist ${NC}"
  #   exit 1;
  # fi
  # if [ ! -d "${media_src_dir}" ]; then
  #   echo "${COL_ERR}[ERROR] ${media_src_dir} does not exist ${NC}"
  #   exit 1;
  # fi
  # if [ ! -d "${logs_src_dir}" ]; then
  #   echo "${COL_ERR}[ERROR] ${logs_src_dir} does not exist ${NC}"
  #   exit 1;
  # fi

  # ensure all data is written before copy
  sync && "${sync_cmd}"

  if [ ${B_CREATE_TARBALL} = true ]; then
    # if localhost, create symlinks to files
    if [ ${on_localhost} = true ]; then
      # link local files accordingly
      ln -s ${local_src_dir}/* ${TAR_TMP_DIR}/
      rm -rf ${TAR_TMP_DIR}/final
      ln -s ${media_src_dir}/* ${TAR_TMP_DIR}/
      rm -rf ${TAR_TMP_DIR}/final
      ln -s ${logs_src_dir}/autonomy/*  ${TAR_TMP_DIR}/logs/autonomy/
      ln -s ${logs_src_dir}/latest/*  ${TAR_TMP_DIR}/logs/ros/

      # Store autonomy logfile with ROS logs for later usage
      ${RSYNC_CMD} ${logs_src_dir}/autonomy/ ${logs_src_dir}/latest/

      # remove source files
      RM_CMD="$RM_CMD $(find ${local_src_dir} -type f -not -path "*/final/*") $(find ${media_src_dir} -type f -not -path "*/final/*") ${logs_src_dir}/autonomy/"
    else
      # copy remote data to local tmp directory
      ${RSYNC_CMD} ${local_src_dir}/ ${media_src_dir}/ ${TAR_TMP_DIR} \
        --exclude='final/'
      ${RSYNC_CMD} ${logs_src_dir}/autonomy ${TAR_TMP_DIR}/logs/
      ${RSYNC_CMD} ${logs_src_dir}/latest/ ${TAR_TMP_DIR}/logs/ros
      ${sync_cmd}

      # Copy data with checks
      ${RSYNC_CMD} -c ${local_src_dir}/ ${media_src_dir}/ ${TAR_TMP_DIR} \
        --exclude='final/'
      ${RSYNC_CMD} -c ${logs_src_dir}/autonomy ${TAR_TMP_DIR}/logs/
      ${RSYNC_CMD} -c ${logs_src_dir}/latest/ ${TAR_TMP_DIR}/logs/ros
      ${sync_cmd}

      # Store autonomy logfile with ROS logs for later usage
      ${RSYNC_CMD} ${logs_src_dir}/autonomy/ ${logs_src_dir}/latest/

      # Copy and delete data
      ${RSYNC_CMD} -c --remove-source-files ${local_src_dir}/ ${media_src_dir}/ ${TAR_TMP_DIR} \
        --exclude='final/'
      ${RSYNC_CMD} -c --remove-source-files ${logs_src_dir}/autonomy ${TAR_TMP_DIR}/logs/
      ${RSYNC_CMD} -c --remove-source-files ${logs_src_dir}/latest/ ${TAR_TMP_DIR}/logs/ros
      ${sync_cmd}
    fi
  else
    # Copy data
    ${RSYNC_CMD} ${local_src_dir}/ ${media_src_dir}/ ${DIR_NAME} \
      --exclude='final/'
    ${RSYNC_CMD} ${logs_src_dir}/autonomy ${DIR_NAME}/logs/
    ${RSYNC_CMD} ${logs_src_dir}/latest/ ${DIR_NAME}/logs/ros
    ${sync_cmd}

    # Copy data with checks
    ${RSYNC_CMD} -c ${local_src_dir}/ ${media_src_dir}/ ${DIR_NAME} \
      --exclude='final/'
    ${RSYNC_CMD} -c ${logs_src_dir}/autonomy ${DIR_NAME}/logs/
    ${RSYNC_CMD} -c ${logs_src_dir}/latest/ ${DIR_NAME}/logs/ros
    ${sync_cmd}

    # Store autonomy logfile with ROS logs for later usage
    ${RSYNC_CMD} ${logs_src_dir}/autonomy/ ${logs_src_dir}/latest/

    # Copy and delete data
    ${RSYNC_CMD} -c --remove-source-files ${local_src_dir}/ ${media_src_dir}/ ${DIR_NAME} \
      --exclude='final/'
    ${RSYNC_CMD} -c --remove-source-files ${logs_src_dir}/autonomy ${DIR_NAME}/logs/
    ${RSYNC_CMD} -c --remove-source-files ${logs_src_dir}/latest/ ${DIR_NAME}/logs/ros
    ${sync_cmd}
  fi
done

# finally create tarball from tmp dir
if [ ${B_CREATE_TARBALL} = true ]; then
  tar --dereference -czvf ${DIR_NAME}.tar.gz -C ${TAR_TMP_DIR}/.. ${TIMESTAMP}
fi
# remove temporary tar dir and local files
$RM_CMD

if [ ${B_DEBUG_ON} = true ]; then
  set +x
fi
