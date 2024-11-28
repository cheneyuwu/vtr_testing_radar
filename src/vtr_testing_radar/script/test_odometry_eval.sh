## this script assumes the following environment variables are set:
##   VTRRROOT VTRRDATA VTRRRESULT
## example usage: test_odometry.sh boreas-2021-09-02-11-42

# Get arguments
ODO_INPUT=$1

# Log
echo "Evaluating odometry of sequence ${ODO_INPUT}, storing result to ${VTRRRESULT}/${ODO_INPUT}/${ODO_INPUT}"

# Source the VTR environment with the testing package
source ${VTRRROOT}/install/setup.bash
source ${VTRROOT}/venv/bin/activate

#   - dump odometry result to boreas expected format (txt file)
python ${VTRRROOT}/src/vtr_testing_radar/script/boreas_generate_odometry_result.py --dataset ${VTRRDATA} --path ${VTRRRESULT}/${ODO_INPUT} --velocity
#   - evaluate the result using the evaluation script
python -m pyboreas.eval.odometry --gt ${VTRRDATA} --pred ${VTRRRESULT}/${ODO_INPUT}/odometry_result --radar --velocity ${VTRRRESULT}/${ODO_INPUT}/odometry_vel_result
