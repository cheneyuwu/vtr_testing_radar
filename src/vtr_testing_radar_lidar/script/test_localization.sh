## this script assumes the following environment variables are set:
##   VTRRROOT VTRRDATA VTRRRESULT
## example usage: test_localization.sh boreas-2021-09-02-11-42 boreas-2021-09-07-09-35

# Get arguments
ODO_INPUT=$1
LOC_INPUT=$2

# Log
echo "Running localization on sequence ${LOC_INPUT} to reference sequence ${ODO_INPUT}, storing result to ${VTRRRESULT}/${ODO_INPUT}/${LOC_INPUT}"

# Source the VTR environment with the testing package
source ${VTRRROOT}/install/setup.bash

rm -r ${VTRRRESULT}/${ODO_INPUT}/${LOC_INPUT}
mkdir -p ${VTRRRESULT}/${ODO_INPUT}/${LOC_INPUT}
cp -r ${VTRRRESULT}/${ODO_INPUT}/${ODO_INPUT}/* ${VTRRRESULT}/${ODO_INPUT}/${LOC_INPUT}
ros2 run vtr_testing_radar_lidar vtr_testing_radar_lidar_boreas_localization \
  --ros-args -p use_sim_time:=true \
  -r __ns:=/vtr \
  --params-file ${VTRRROOT}/src/vtr_testing_radar_lidar/config/boreas.yaml \
  -p data_dir:=${VTRRRESULT}/${ODO_INPUT}/${LOC_INPUT} \
  -p odo_dir:=${VTRRDATA}/${ODO_INPUT} \
  -p loc_dir:=${VTRRDATA}/${LOC_INPUT}
