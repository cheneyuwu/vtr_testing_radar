## this script assumes the following environment variables are set:
##   VTRRROOT VTRRDATA VTRRRESULT
## example usage: test_odometry.sh boreas-2021-09-02-11-42

# Get arguments
ODO_INPUT=$1
if [ $# -eq 2 ]; then
  PARAM_FILE=$2
else
  PARAM_FILE=${VTRRROOT}/src/vtr_testing_radar/config/boreas.yaml
fi

# Log
echo "Running odometry on sequence ${ODO_INPUT}, storing result to ${VTRRRESULT}/${ODO_INPUT}/${ODO_INPUT}"

# Source the VTR environment with the testing package
source ${VTRRROOT}/install/setup.bash

graph_dir=${VTRRRESULT}/${ODO_INPUT}/${ODO_INPUT}/graph
if [ -d $graph_dir ]; then
  # Count the number of directories inside "graph"
  dir_count=$(ls -l $graph_dir | grep -c ^d)
  
  if [ $dir_count -gt 1 ]; then
    read -p "The directory $graph_dir is not empty and contains $dir_count other directories. Do you want to delete it and create an empty one? (yes/no) " response
    if [ "$response" == "no" ]; then
      exit
    fi
  fi
fi

ros2 run vtr_testing_radar vtr_testing_radar_boreas_odometry \
  --ros-args -p use_sim_time:=true \
  -r __ns:=/vtr \
  --params-file ${PARAM_FILE} \
  -p data_dir:=${VTRRRESULT}/${ODO_INPUT}/${ODO_INPUT} \
  -p odo_dir:=${VTRRDATA}/${ODO_INPUT}
