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

################################################################################
# Global Variables                                                             #
################################################################################

# setup colors
COL_ERR='\033[0;31m'  #Red Color
COL_WARN='\033[0;33m' #Yellow Color
COL_DEB='\033[0;32m'  #Green Color
NC='\033[0m'          #No Color

# SCRIPT VARIABLES
CONFIG_NAME="myconfig"
PATH_WS="${HOME}/"

# INPUT FLAGS
PATH_PROVIDED=false
CONFIG_PROVIDED=false
CREATE_GIT=true
VERBOSE_ON=false

################################################################################
# Help                                                                         #
################################################################################
print_help(){
    echo "USAGE: ${script_name} [OPTIONS]"
    echo ""
    echo "  Generates a configuration workspace for the CNS Flight Stack with ready-to-use and ready-to-edit "
    echo "  configuration files."
    echo ""
    echo "  Options:"
    echo "    -d PATH       path to the desired new workspace, default '~/'"
    echo "    -n NAME       name of the configuration package, default 'myconfig'"
    echo "    -g            disable creation of automatic git repo"
    echo "    -v            verbose debug output"
    echo ""
    echo "    -h        print this help"
    echo ""
    exit 0;
}

################################################################################
# Execution Options                                                            #
################################################################################

# parse flags
while getopts vhgn:d: flag
do
    case "${flag}" in
        d) PATH_WS=${OPTARG}; PATH_PROVIDED=true;;
        n) CONFIG_NAME=${OPTARG}; CONFIG_PROVIDED=true;;

        g) CREATE_GIT=false;;
        h) print_help;;
        v) VERBOSE_ON=true;;

        *) echo "Unknown option ${flag}"; print_help;;
    esac
done
shift $((OPTIND-1))

# output warnings
if [[ "${PATH_PROVIDED}" = false ]]; then
  echo -e "${COL_WARN}[WARN ] no path provided, using '${PATH_WS}'${NC}"
  echo -e "    see '${script_name} -h' for further information"
fi
if [[ "${CONFIG_PROVIDED}" = false ]]; then
  echo -e "${COL_WARN}[WARN ] no config name provided, using '${CONFIG_NAME}'${NC}"
  echo -e "    see '${script_name} -h' for further information"
fi

# turn on command output
if [[ "${VERBOSE_ON}" = true ]]; then
  echo -e "${COL_DEB}[DEBUG] verbose turned on${NC}"
  set -x
fi

################################################################################
# MAIN SCRIPT                                                                  #
################################################################################

# setup workspace dir
FLIGHTSTACK_DEVEL=$(pwd)/devel
WORKSPACE_DIR=${PATH_WS}/${CONFIG_NAME}_cws/
WORKSPACE_SRC=${PATH_WS}/${CONFIG_NAME}_cws/src
PACKAGE_DIR=${PATH_WS}/${CONFIG_NAME}_cws/src/${CONFIG_NAME}
BRINGUP_DIR=${PATH_WS}/${CONFIG_NAME}_cws/src/${CONFIG_NAME}/${CONFIG_NAME}_bringup

# check if executed in correct folder
if [ ! -d 'scripts' ]; then
  echo "${COL_ERR}No 'scripts' folder found, please execute this script in the root directory of the flightstack_cws.${NC}"
  exit 1;
elif [ ! -f 'scripts/create_bringup.sh' ]
  echo "${COL_ERR}No 'create_bringup' script found within the './scripts' folder, please execute this script in the root directory of the flightstack_cws.${NC}"
  exit 1;
fi

# create directory and copy template
mkdir -p ${PACKAGE_DIR}
rsync -avPh $(pwd)/src/flightstack/template_config/ ${PACKAGE_DIR}

# replace 'template' directories with config name
find ${PACKAGE_DIR}/ -depth -type d -name '*template*' -execdir bash -c 'mv -v "$1" "${1/template/'${CONFIG_NAME}'}"' _ {} \;

# replace 'template' files with config name
find ${PACKAGE_DIR}/ -type f -name '*template*' | while read FILE ; do
  # echo "File to replace: ${FILE}"
  newfile="$(echo ${FILE} |sed -e 's/template/'${CONFIG_NAME}'/')" ;
  mv "${FILE}" "${newfile}" ;
done

# replace 'template' in files with config name
find ${PACKAGE_DIR}/ -type f -exec sed -i "s/template/${CONFIG_NAME}/g" {} \;

# create git repo and copy gitignore
rsync -avPh $(pwd)/.gitignore ${WORKSPACE_DIR}/
if [[ "${CREATE_GIT}" = true ]]; then
  /bin/bash -c "cd ${WORKSPACE_DIR}; \
                git init; git checkout -b main; git add .; git commit -m 'init: automatic initial commit'"
fi

# compile workspace
/bin/bash -c "cd ${WORKSPACE_DIR}; \
              catkin init; \
              catkin config --extend ${FLIGHTSTACK_DEVEL} -j`nproc --ignore=1` --cmake-args -DCMAKE_BUILD_TYPE=Release; \
              catkin build"


################################################################################
# Cleanup                                                                      #
################################################################################

# turn of command output
if [[ "${VERBOSE_ON}" = true ]]; then
  set +x
fi
