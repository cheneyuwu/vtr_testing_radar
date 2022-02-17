# Experiments on Obelisk

First, `ssh` into obelisk!

## Setup VTR3 Directories

Create the following directories. I suggest you don't change any of them.

```
export VTRROOT=~/ASRL  # (INTERNAL default) root directory of VTR3
# you can change the following directories to anywhere appropriate
export VTRSRC=${VTRROOT}/vtr3        # source code of VTR3 (this repo)
export VTRDEPS=${VTRROOT}/deps       # system dependencies of VTR3
export VTRVENV=${VTRROOT}/venv       # python dependencies of VTR3 (not used at the moment)
export VTRDATA=${VTRROOT}/data       # datasets for VTR3
export VTRTEMP=${VTRROOT}/temp       # temporary data directory for testing
mkdir -p ${VTRSRC} ${VTRDEPS} ${VTRVENV} ${VTRDATA} ${VTRTEMP}
```

## Download VTR3 Source Code

Clone your code into `VTRSRC`.

Remember to check out the branch you are working on!
Eventually we will merge all changes back to `main_lidar` branch (including radar-radar and radar-lidar)

Currently:

Yuchen - using main_lidar branch to run experiments of radar-lidar

Keenan - using main_radar branch to improve radar-radar

Here's the command:

```
cd ${VTRSRC}
git clone git@github.com:utiasASRL/vtr3.git .
git checkout main_lidar  # CHECK OUT THE BRANCH YOU ARE WORKING ON!
git submodule update --init --remote
```

## Download vtr_testing_radar repo

ALL the testing scripts are in this repo, in the `src` directory, you can see three ros2 packages - they are for lidar-lidar, radar-radar and radar-lidar, respectively.

Use the following command to download the package into `VTRROOT`.

```
cd ${VTRROOT}
git clone git@github.com:cheneyuwu/vtr_testing_radar.git
```

## Download pyboreas for evaluation

For now, you will need to use the `localization_eval` branch of pyboreas. Clone it

It doesn't matter where you clone the source code into, so at any directory:

```
git clone git@github.com:utiasASRL/pyboreas.git .
git checkout localization_eval
```

Remember this directory.


## Build vtr3 docker image

We run experiments in a docker container on obelisk. Here is how to build the image:

PLEASE CHANGE THE IMAGE NAME TO `vtr3_main_lidar_<your name>` TO AVOID CONFLICTS.

```
cd ${VTRSRC}
docker build -t vtr3_main_lidar \
  --build-arg USERID=$(id -u) \
  --build-arg GROUPID=$(id -g) \
  --build-arg USERNAME=$(whoami) \
  --build-arg HOMEDIR=${HOME} .
```

## Start the VTR3 Docker container

Note the name of your image and container may be different.

```
docker run -it --rm --name vtr3_main_lidar \
  --privileged \
  --network=host \
  -v ${HOME}/ASRL:${HOME}/ASRL:rw vtr3_main_lidar
```

FYI: to start a new terminal with the existing container: `docker exec -it --privileged vtr3_main_lidar bash`

## Build and Install VT&R3

Start a new terminal (**terminal 1**) and **enter the container**.

Use the following command to compile the code.

NOTE: if you only need to run radar-radar experiments, then in you only need to build package up to `vtr_radar`. Use the `--packages-up-to` option of `colcon build` to do this.

```
source /opt/ros/galactic/setup.bash  # source the ROS environment
cd ${VTRSRC}/main
colcon build --symlink-install --packages-up-to vtr_radar
```

wait until it finishes.


# Build and Install vtr_testing_radar (this package)

Start a new terminal (**terminal 2**) and **enter the container**

Use the following command to compile the code.

NOTE: again, if you onlhy need to run radar-radar experiements then you only need to build `vtr_testing_radar`. Use the `--packages-select` option of `colcon build` to do this

```
source ${VTRSRC}/main/install/setup.bash # source the vtr3 environment
cd ${VTRROOT}/vtr_testing_radar # go to where this repo is located
colcon build --symlink-install  --packages-select vtr_testing_radar
```

wait until it finishes.

NOTE: whenever you change any code in the vtr3 repo, you need to re-compile and re-install, do this by going to terminal 1 & 2 and re-running the `colcon build ....` command, respectively. Always wait until build process in terminal 1 finishes before running command in terminal 2.

## Create a python venv to install pyboreas

Start a new terminal (**terminal 3**) and **enter the container**

Create a virtual environment at `${VTRROOT}`

```
cd ${VTRROOT}
virtualenv venv
source venv/bin/activate  # activate this environment
```

Install pyboreas `localization_eval` branch

```
cd <where you downloaded pyboreas to>
pip install -e .
```

## Run Tests

Assuming you want to run odometry or localization for all test sequences in parallel.

Inside the `script` folder of all three testing packages (`vtr_testing_<...>`), you can find the following two bash script:

- `parallel_test_odometry.sh`
- `parallel_test_localization.sh`


All you need to do is run one of the above bash scripts **inside the contaner**. 

```
bash <path to parallel_test_odometry.sh or parallel_test_localization.sh>
```

Then monitor progress by going to the log file of each test.

The log file should be located at

`~/ASRL/temp/[radar, lidar, radar_lidar]/boreas/<boreas-2020-11-26-13-58>/<boreas-2020-11-26-13-58>/<some name based on time>.log`

Understand what these scripts do:

Using `parallel_test_odometry.sh` from `src/vtr_testing_radar/script` as an example, the script does the following:

1. Define sequences we need to run for odometry

```
# odometry sequences
SEQUENCES=(
  'boreas-2020-11-26-13-58'  # Note this is the localization reference run, you must run this in order to run localization tests
  'boreas-2020-12-04-14-00'
  'boreas-2021-01-26-10-59'
  'boreas-2021-02-09-12-55'
  'boreas-2021-03-09-14-23'
  'boreas-2021-04-22-15-00'
  'boreas-2021-06-29-18-53'
  'boreas-2021-06-29-20-43'
  'boreas-2021-09-08-21-00'
  # 'boreas-2021-09-09-15-28'  # this is a st george run
  # the following runs use a new radar, not working...
  # 'boreas-2021-10-05-15-35'
  # 'boreas-2021-10-26-12-35'
  # 'boreas-2021-11-06-18-55'
  # 'boreas-2021-11-28-09-18'
)
```

2. Set max number of sequences to run in parallel

```
# maximum number of jobs running in parallel
GROUPSIZE=20
```

3. Setup up directories

These directories are defined using the environment variables in `Setup VTR3 Directories` section.

I suggest you don't change them.

For `VTRRDATA`, it is supposed to be the directory that contains all boreas sequences (i.e. `boreas-....`). You can create a symlink from boreas dataset on /nas to this directory.

```
# define the following environment variables VTRR=VTR RaDAR
export VTRRROOT=${VTRROOT}/vtr_testing_radar # location of this repository CHANGE THIS!
export VTRRDATA=${VTRDATA}/boreas/sequences  # dataset location (where the boreas-xxxxx folders at) CHANGE THIS!
export VTRRRESULT=${VTRTEMP}/radar/boreas    # result location MAYBE CHANGE THIS!
mkdir -p ${VTRRRESULT}
```

4. Define path to test scripts

```
ODOMETRY_SCRIPT="${VTRRROOT}/src/vtr_testing_radar/script/test_odometry.sh"
ODOMETRY_EVAL_SCRIPT="${VTRRROOT}/src/vtr_testing_radar/script/test_odometry_eval.sh"
```

These are bash scripts that will run odometry test (using `ros2 run ...`) and evaluation.

5. Run odometry tests in parallel

The following code runs at most `GROUPSIZE` odometry tests in parallel by calling the `$ODOMETRY_SCRIPT` test script with each of the sequence specified in `SEQUENCES`.

```
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
```

6. Run evaluation

When all sequences are finished, the following code runs pyboreas odometry evaluation on the result of each sequence. You should see output in terminal.

```
for seq in ${SEQUENCES[@]}; do
  echo "Executing command: bash $ODOMETRY_EVAL_SCRIPT $seq"
  bash $ODOMETRY_EVAL_SCRIPT $seq
done
```