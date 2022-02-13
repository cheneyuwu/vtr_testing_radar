## this script assumes the following environment variables are set:
##   VTRRROOT VTRRDATA VTRRRESULT
## example usage: test_preprocessing.sh boreas-2021-09-02-11-42

# Get arguments
ODO_INPUT=$1

# Log
echo "Running preprocessing on sequence ${ODO_INPUT}, storing result to ${VTRRRESULT}/${ODO_INPUT}/${ODO_INPUT}"

# Source the VTR environment with the testing package
source ${VTRRROOT}/install/setup.bash

ros2 run vtr_testing_radar vtr_testing_radar_boreas_preprocessing \
  --ros-args -p use_sim_time:=true \
  -r __ns:=/vtr \
  --params-file ${VTRRROOT}/src/vtr_testing_radar/config/boreas.yaml \
  -p data_dir:=${VTRRRESULT}/${ODO_INPUT}/${ODO_INPUT} \
  -p odo_dir:=${VTRRDATA}/${ODO_INPUT}
# Explanation:
#   - this script will load all radar data in time order and feed it to the preprocessing pipeline, which will run the keypoint extraction module.
#   - in the rviz window, change `Global Options -> Fixed Frame` to "radar", and look at the following:
#      - raw scan: ROS Image of the raw radar scan (from cv::imread)
#      - fft scan: output from load_radar function
#      - raw point cloud: output from keypoint detector
#     see if these visualization are expected