echo "This script contains instructions to run all tests, do not run this script directly."
exit 1

## First launch RViz for visualization
source /opt/ros/galactic/setup.bash  # source the ROS environment
ros2 run rviz2 rviz2 -d ${VTRSRC}/rviz/radar.rviz  # launch rviz

############################################################
#### Now start another terminal and run testing scripts ####


## Terminal Setup (Run Following Once)

# Define the following environment variables VTRR=VTR RaDAR
export VTRRROOT=/ext0/ASRL/vtr_testing_radar    # location of this repository CHANGE THIS!
export VTRRDATA=${VTRDATA}/boreas/sequences     # dataset location (where the boreas-xxxxx folders at) CHANGE THIS!
export VTRRRESULT=${VTRTEMP}/radar/boreas       # result location MAYBE CHANGE THIS!
mkdir -p ${VTRRRESULT}

# Choose a Teach (ODO_INPUT) and Repeat (LOC_INPUT) run from boreas dataset
ODO_INPUT=boreas-2021-09-02-11-42
LOC_INPUT=boreas-2021-09-07-09-35

# Source the VTR environment with the testing package
source ${VTRRROOT}/install/setup.bash


## Using ONE of the following commands to launch a test

# (TEST 1) Perform data preprocessing on a sequence (e.g. keypoint extraction)
ros2 run vtr_testing_radar vtr_testing_radar_boreas_preprocessing  \
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

# (TEST 2) Perform odometry on a sequence
ros2 run vtr_testing_radar vtr_testing_radar_boreas_odometry  \
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
# Evaluation:
#   - dump odometry result to boreas expected format (txt file)
python ${VTRRROOT}/src/vtr_testing_radar/script/boreas_generate_odometry_result.py --dataset ${VTRRDATA} --path ${VTRRRESULT}/${ODO_INPUT}
#   - evaluate the result using the evaluation script
python -m pyboreas.eval.odometry --gt ${VTRRDATA}  --pred ${VTRRRESULT}/${ODO_INPUT}/odometry_result --radar

# (TEST 3) Perform localization on a sequence (only run this after TEST 2)
cp -r ${VTRRRESULT}/${ODO_INPUT}/${ODO_INPUT}  ${VTRRRESULT}/${ODO_INPUT}/${ODO_INPUT}.bak
rm -r ${VTRRRESULT}/${ODO_INPUT}/${LOC_INPUT}
mkdir -p ${VTRRRESULT}/${ODO_INPUT}/${LOC_INPUT}
cp -r ${VTRRRESULT}/${ODO_INPUT}/${ODO_INPUT}/*  ${VTRRRESULT}/${ODO_INPUT}/${LOC_INPUT}
ros2 run vtr_testing_radar vtr_testing_radar_boreas_localization  \
  --ros-args -p use_sim_time:=true \
  -r __ns:=/vtr \
  --params-file ${VTRRROOT}/src/vtr_testing_radar/config/boreas.yaml \
  -p data_dir:=${VTRRRESULT}/${ODO_INPUT}/${LOC_INPUT} \
  -p odo_dir:=${VTRRDATA}/${ODO_INPUT} \
  -p loc_dir:=${VTRRDATA}/${LOC_INPUT}
# # Evaluation:
# #   - dump localization result to boreas expected format (txt file)
# python ${VTRRROOT}/src/vtr_testing_radar/script/boreas_generate_localization_result.py --dataset ${VTRRDATA} --path ${VTRRRESULT}/${ODO_INPUT}
# #   - evaluate the result using the evaluation script
# python -m pyboreas.eval.localization --gt ${VTRRDATA}  --pred ${VTRRRESULT}/${ODO_INPUT}/localization_result --ref_seq ${ODO_INPUT} --ref_sensor radar --radar