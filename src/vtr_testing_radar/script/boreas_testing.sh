## Explain how to test radar pipeline

## First launch RViz for visualization
source /opt/ros/galactic/setup.bash  # source the ROS environment
ros2 run rviz2 rviz2 -d ${VTRSRC}/rviz/radar.rviz  # launch rviz

## Now start another terminal and run testing scripts

# Define the following environment variables VTRB=VTR RaDAR
export VTRRROOT=/ext0/ASRL/vtr_testing_radar/src/vtr_testing_radar    # location of this package CHANGE THIS!
export VTRRDATA=${VTRDATA}/boreas/sequences                           # dataset location (where the boreas-xxxxx folders at) CHANGE THIS!
export VTRRRESULT=${VTRTEMP}/radar/boreas                             # result location MAYBE CHANGE THIS!
mkdir -p ${VTRRRESULT}

# Choose a Teach (ODO_INPUT) and Repeat (LOC_INPUT) run from boreas dataset
ODO_INPUT=boreas-2021-11-02-11-16
LOC_INPUT=boreas-2021-11-02-11-16

# Source the VTR environment with the testing package
source ${VTRRROOT}/../../install/setup.bash

# (TEST 1) Perform data preprocessing on a sequence (e.g. keypoint extraction)
ros2 run vtr_testing_radar vtr_testing_radar_boreas_preprocessing  \
  --ros-args  -r __ns:=/vtr  --params-file ${VTRRROOT}/config/boreas.yaml \
  -p data_dir:=${VTRRRESULT}/${ODO_INPUT}/${ODO_INPUT} \
  -p odo_dir:=${VTRRDATA}/${ODO_INPUT}
# Explanation:
#   - this script will load all radar data in time order and feed it to the preprocessing pipeline, which will run the keypoint extraction module.
#   - in the rviz window, change `Global Options -> Fixed Frame` to "radar", and look at the following:
#      - raw scan: ROS Image of the raw radar scan (from cv::imread)
#      - fft scan: output from load_radar function
#      - raw point cloud: output from keypoint detector
#     see if these visualization are expected