echo "This script contains instructions to run all tests, do not run this script directly."
exit 1

## First launch RViz for visualization
source /opt/ros/galactic/setup.bash  # source the ROS environment
ros2 run rviz2 rviz2 -d ${VTRSRC}/rviz/lidar.rviz  # launch rviz

############################################################
#### Now start another terminal and run testing scripts ####


## Terminal Setup (Run Following Once)

# Define the following environment variables VTRR=VTR RaDAR
export VTRRROOT=/ext0/ASRL/vtr_testing_radar    # location of this repository CHANGE THIS!
export VTRRDATA=${VTRDATA}/boreas/sequences     # dataset location (where the boreas-xxxxx folders at) CHANGE THIS!
export VTRRRESULT=${VTRTEMP}/lidar/boreas       # result location MAYBE CHANGE THIS!
mkdir -p ${VTRRRESULT}

# Choose a Teach (ODO_INPUT) and Repeat (LOC_INPUT) run from boreas dataset
ODO_INPUT=boreas-2020-12-01-13-26
LOC_INPUT=boreas-2020-12-01-13-26

# Source the VTR environment with the testing package
source ${VTRRROOT}/install/setup.bash


## Using ONE of the following commands to launch a test

# (TEST 1) Perform data preprocessing on a sequence (e.g. keypoint extraction)
ros2 run vtr_testing_lidar vtr_testing_lidar_boreas_preprocessing  \
  --ros-args -p use_sim_time:=true \
  -r __ns:=/vtr \
  --params-file ${VTRRROOT}/src/vtr_testing_lidar/config/boreas.yaml \
  -p data_dir:=${VTRRRESULT}/${ODO_INPUT}/${ODO_INPUT} \
  -p odo_dir:=${VTRRDATA}/${ODO_INPUT}

# (TEST 2) Perform odometry on a sequence
ros2 run vtr_testing_lidar vtr_testing_lidar_boreas_odometry  \
  --ros-args -p use_sim_time:=true \
  -r __ns:=/vtr \
  --params-file ${VTRRROOT}/src/vtr_testing_lidar/config/boreas.yaml \
  -p data_dir:=${VTRRRESULT}/${ODO_INPUT}/${ODO_INPUT} \
  -p odo_dir:=${VTRRDATA}/${ODO_INPUT}
# Evaluation:
#   - dump odometry result to boreas expected format (txt file)
python ${VTRRROOT}/src/vtr_testing_lidar/script/boreas_generate_odometry_result.py --dataset ${VTRRDATA} --path ${VTRRRESULT}/${ODO_INPUT}
#   - evaluate the result using the evaluation script
python -m pyboreas.eval.odometry --gt ${VTRRDATA}  --pred ${VTRRRESULT}/${ODO_INPUT}