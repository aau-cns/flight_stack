#! /bin/bash


git submodule update --init --recursive

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
CUR_DIR=${pwd}
cd $DIR/src

echo "catkin_init workspace"
catkin_init_workspace


cd ${CUR_DIR}
