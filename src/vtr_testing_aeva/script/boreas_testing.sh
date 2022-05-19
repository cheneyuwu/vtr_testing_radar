echo "This script contains instructions to run all tests, do not run this script directly."
exit 1

## First launch RViz for visualization
source /opt/ros/galactic/setup.bash               # source the ROS environment
ros2 run rviz2 rviz2 -d ${VTRSRC}/rviz/lidar.rviz # launch rviz

## Then in another terminal, launch rqt_reconfigure for control
## current supported dynamic reconfigure parameters: control_test.play and controL_test.delay_millisec
source /opt/ros/galactic/setup.bash
ros2 run rqt_reconfigure rqt_reconfigure

############################################################
#### Now start another terminal and run testing scripts ####

## Terminal Setup (Run Following Once)

# Define the following environment variables VTRR=VTR RaDAR
export VTRRROOT=${VTRROOT}/vtr_testing_radar # location of this repository CHANGE THIS!
export VTRRDATA=${VTRDATA}/boreas/sequences  # dataset location (where the boreas-xxxxx folders at) CHANGE THIS!
export VTRRRESULT=${VTRTEMP}/lidar/aeva      # result location MAYBE CHANGE THIS!
mkdir -p ${VTRRRESULT}

# Source the VTR environment with the testing package
source ${VTRRROOT}/install/setup.bash

# Choose a Teach (ODO_INPUT) and Repeat (LOC_INPUT) run from boreas dataset
ODO_INPUT=boreas-2022-05-06-15-22
ODO_INPUT=boreas-2022-05-13-09-23  # hwy7 first sequence
ODO_INPUT=boreas-2022-05-13-09-45  # hwy7 second sequence (ground truth is probably bad)
ODO_INPUT=boreas-2022-05-13-10-21  # dufferin first sequence (no ground truth)
ODO_INPUT=boreas-2022-05-13-10-30  # dufferin second sequence
ODO_INPUT=boreas-2022-05-13-11-47

## Using ONE of the following commands to launch a test

# (TEST 1) Perform data preprocessing on a sequence (e.g. keypoint extraction)
bash ${VTRRROOT}/src/vtr_testing_aeva/script/test_preprocessing.sh ${ODO_INPUT}

# (TEST 2) Perform odometry on a sequence (this includes preprocessing, no need to run TEST1 first)
bash ${VTRRROOT}/src/vtr_testing_aeva/script/test_odometry.sh ${ODO_INPUT} ; alert
# Evaluation:
bash ${VTRRROOT}/src/vtr_testing_aeva/script/test_odometry_eval.sh ${ODO_INPUT}

