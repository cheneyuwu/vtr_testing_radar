## this script assumes the following environment variables are set:
##   VTRRROOT VTRRDATA VTRRRESULT
## example usage: test_localization.sh boreas-2021-09-02-11-42 boreas-2021-09-07-09-35

# Get arguments
ODO_INPUT=$1
TYPE=$2

# Log
echo "Evaluating localization to reference sequence ${ODO_INPUT}, storing result to ${VTRRRESULT}/${ODO_INPUT}"

# Source the VTR environment with the testing package
source ${VTRRROOT}/install/setup.bash
source ${VTRROOT}/venv/bin/activate

#   - dump localization result to boreas expected format (txt file)
if [ "$TYPE" = "boreas_aeva" ]; then
    python ${VTRRROOT}/src/vtr_testing_aeva/script/boreas_generate_localization_result.py --dataset ${VTRRDATA} --path ${VTRRRESULT}/${ODO_INPUT}
elif [ "$TYPE" = "aeva_hq" ]; then
    python ${VTRRROOT}/src/vtr_testing_aeva/script/aevahq_generate_localization_result.py --dataset ${VTRRDATA} --path ${VTRRRESULT}/${ODO_INPUT}
fi

python ${VTRRROOT}/src/vtr_testing_aeva/script/boreas_generate_localization_result.py --dataset ${VTRRDATA} --path ${VTRRRESULT}/${ODO_INPUT}
#   - evaluate the result using the evaluation script
python -m pyboreas.eval.localization_aeva --gt ${VTRRDATA} --pred ${VTRRRESULT}/${ODO_INPUT}/localization_result --ref_seq ${ODO_INPUT} --data_type ${TYPE} --ref_sensor aeva --test_sensor aeva --dim 3  --plot ${VTRRRESULT}/${ODO_INPUT}/localization_result/lidar-lidar
