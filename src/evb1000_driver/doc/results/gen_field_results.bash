#! /bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
CUR_DIR=${pwd}
cd $DIR

calc(){ echo "scale=2;$@" | bc;}
#git clone -b 72f6aa698099f589416beb03888ce2c50fc085d3 git@gitlab.aau.at:aau-nav/development/aaunav_data_analysis_py.git
#DATA_ANALYSIS_PY_DIR="${DIR}/aaunav_data_analysis_py"
DATA_ANALYSIS_PY_DIR="/home/jungr/workspace/NAV/development/aaunav_data_analysis_py"
TYPE=field

declare -a list_DIST_CM=("101" "263" "540" "1330" "3310" "6200" "12400")
files=()
length=${#list_DIST_CM[@]}
distances_m=()
for (( i=0; i < ${length}; i++ ));
do
  DIST_CM=${list_DIST_CM[$i]}
  DIST_M=$(echo "scale=2;(${DIST_CM}/100)" | bc)

  file="../measurements/EVK1000/${TYPE}/recording_${DIST_CM}cm.csv"
  files+=(${file})
  distances_m+=(${DIST_M})
  python ${DATA_ANALYSIS_PY_DIR}/data_analysis/static_precision.py --sample ${file} --save ./${TYPE}/${TYPE}_${DIST_CM}cm.png --title "narrow_${DIST_CM}cm" --verbose --reference ${DIST_M} ${DIST_M}
  
  python ${DATA_ANALYSIS_PY_DIR}/data_analysis/analyse_timestamp.py --verbose --trajectories ../measurements/EVK1000/${TYPE}/recording_${DIST_CM}cm.csv --save ./${TYPE}/${TYPE}_${DIST_CM}cm.dts.png
done

#echo "files: " ${files[@]}

python ${DATA_ANALYSIS_PY_DIR}/data_analysis/plot_errorbar1D.py --measurements ${files[@]} --values ${distances_m[@]} --save ./${TYPE}/${TYPE}_error_bar.png  --verbose --select tx --x_scale linear --fill

python ${DATA_ANALYSIS_PY_DIR}/data_analysis/plot_trajectories.py --trajectories ../measurements/EVK1000/${TYPE}/recording_walk.csv  --show --save ./${TYPE}/walk.png  --title "field walk" --verbose --type time2D --filter y z

cd $CUR_DIR
