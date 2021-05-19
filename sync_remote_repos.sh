#! /bin/bash
#description     :This script will synchronize the src directory with the remote directories
#                : remotes are amadee1 and amadee2 hosts
#author		 :Roland Jung
#date            :2021-05-21
#usage		 :sync_remote_repos.sh



DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
CUR_DIR=${pwd}
cd $DIR


echo "catkin_init workspace"
./init.sh

echo "sync with AMADEE1..."
rsync -ral --info=progress1 -c ./src amadee1_w_core:/home/core/catkin_ws/ 
echo "sync with AMADEE2..."
rsync -ral --info=progress2 -c ./src amadee2_w_core:/home/core/catkin_ws/ 


echo "DONE"

cd ${CUR_DIR}

