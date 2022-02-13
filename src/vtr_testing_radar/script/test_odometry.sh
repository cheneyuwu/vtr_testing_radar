## this script assumes the following environment variables are set:
##   VTRRROOT VTRRDATA VTRRRESULT
## example usage: test_odometry.sh boreas-2021-09-02-11-42

# Get arguments
ODO_INPUT=$1

# Log
echo "Running odometry on sequence ${ODO_INPUT}, storing result to ${VTRRRESULT}/${ODO_INPUT}/${ODO_INPUT}"

# Source the VTR environment with the testing package
source ${VTRRROOT}/install/setup.bash

ros2 run vtr_testing_radar vtr_testing_radar_boreas_odometry \
  --ros-args -p use_sim_time:=true \
  -r __ns:=/vtr \
  --params-file ${VTRRROOT}/src/vtr_testing_radar/config/boreas.yaml \
  -p data_dir:=${VTRRRESULT}/${ODO_INPUT}/${ODO_INPUT} \
  -p odo_dir:=${VTRRDATA}/${ODO_INPUT}
# Explanation:
#   - this script will load all radar data in time order and feed it to the preprocessing+odometry pipeline
#   - in the rviz window, change `Global Options -> Fixed Frame` to "world", and look at the following:
#      - undistorted point cloud: output from ICP module, which uses STEAM to undistort the point cloud
#      - curr map odo: current map for odometry
#      - odo path: history of robot poses at every radar frame as ros odometry msg
