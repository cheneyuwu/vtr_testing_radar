# odometry sequences
SEQUENCES=(
  'boreas-2020-11-26-13-58'
  'boreas-2020-12-04-14-00'
  'boreas-2021-01-26-10-59'
  'boreas-2021-02-09-12-55'
  'boreas-2021-03-09-14-23'
  'boreas-2021-04-22-15-00'
  'boreas-2021-06-29-18-53'
  'boreas-2021-06-29-20-43'
  'boreas-2021-09-08-21-00'
  'boreas-2021-09-09-15-28'
  # the following runs use a new radar, not working...
  # 'boreas-2021-10-05-15-35'
  # 'boreas-2021-10-26-12-35'
  # 'boreas-2021-11-06-18-55'
  # 'boreas-2021-11-28-09-18'
)

# maximum number of jobs running in parallel
GROUPSIZE=10

# define the following environment variables VTRR=VTR RaDAR
export VTRRROOT=${VTRROOT}/vtr_testing_radar # location of this repository CHANGE THIS!
export VTRRDATA=${VTRDATA}/boreas/sequences  # dataset location (where the boreas-xxxxx folders at) CHANGE THIS!
export VTRRRESULT=${VTRTEMP}/radar/boreas    # result location MAYBE CHANGE THIS!
mkdir -p ${VTRRRESULT}

ODOMETRY_SCRIPT="${VTRRROOT}/src/vtr_testing_radar/script/test_odometry.sh"
ODOMETRY_EVAL_SCRIPT="${VTRRROOT}/src/vtr_testing_radar/script/test_odometry_eval.sh"

declare -A pids

for seq in ${SEQUENCES[@]}; do
  echo "Executing command: bash $ODOMETRY_SCRIPT $seq &>/dev/null &"
  ### command to execute
  bash $ODOMETRY_SCRIPT $seq &>/dev/null &
  ###
  pids[${seq}]=$!
  # wait for all pids to finish if reached group size
  if [[ ${#pids[@]} -ge ${GROUPSIZE} ]]; then
    for key in ${!pids[@]}; do
      wait ${pids[${key}]}
      echo "Process ${key} finished with return code ${?}"
      unset pids[${key}]
    done
  fi
done
for key in ${!pids[@]}; do
  wait ${pids[${key}]}
  echo "Process ${key} finished with return code ${?}"
  unset pids[${key}]
done

for seq in ${SEQUENCES[@]}; do
  echo "Executing command: bash $ODOMETRY_EVAL_SCRIPT $seq"
  bash $ODOMETRY_EVAL_SCRIPT $seq
done