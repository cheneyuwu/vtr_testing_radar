# Installation

Instructions to install vtr3 main_lidar branch using docker container.

## Setup VTR3 Directories

Create the following directories in your local filesystem. Later they will be mapped to the docker container.

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

Reference: https://github.com/utiasASRL/vtr3/wiki/Installation-Guide

## Download VTR3 Source Code

Also to your local filesystem, so that you don't have to access them from within the docker container.

```
cd ${VTRSRC}
git clone git@github.com:utiasASRL/vtr3.git .
git checkout main_lidar  # use the main_lidar branch
git submodule update --init --remote
```

Reference: https://github.com/utiasASRL/vtr3/wiki/Installation-Guide

## Download this package

This package contains testing code for lidar and radar pipeline. Download it do your local filesystem.

```
cd ${VTRROOT}
git clone git@github.com:cheneyuwu/vtr_testing_radar.git
```

## Build VTR3 Docker Image

This builds a image that has all dependencies installed.

NOTE that if you are using obelisk, then change the image name to `vtr3_lidar_<your name>` or something else to avoid conflicts.

```
cd ${VTRSRC}
docker build -t vtr3_lidar \
  --build-arg USERID=$(id -u) \
  --build-arg GROUPID=$(id -g) \
  --build-arg USERNAME=$(whoami) \
  --build-arg HOMEDIR=${HOME} .
```

Reference: https://github.com/utiasASRL/vtr3/wiki/EXPERIMENTAL-Running-VTR3-from-a-Docker-Container

## Start the VTR3 Docker container

Install nvidia docker runtime first: https://nvidia.github.io/nvidia-container-runtime/

```
docker run -it --name vtr3_lidar \
  --privileged \
  --network=host \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ${HOME}:${HOME}:rw \
  -v ${HOME}/ASRL:${HOME}/ASRL:rw vtr3_lidar
```

FYI: to start a new terminal with the existing container: `docker exec -it --privileged vtr3_lidar bash`

Reference: https://github.com/utiasASRL/vtr3/wiki/EXPERIMENTAL-Running-VTR3-from-a-Docker-Container

## Build and Install VT&R3

Start a new terminal (**terminal 1**) and enter the container

```
source /opt/ros/galactic/setup.bash  # source the ROS environment
cd ${VTRSRC}/main
colcon build --symlink-install --packages-up-to vtr_lidar vtr_radar # only build up to the vtr_lidar/vtr_radar package
```

wait until it finishes.

# Build and Install vtr_testing_radar (this package)

Start a new terminal (**terminal 2**) and enter the container

```
source ${VTRSRC}/main/install/setup.bash # source the vtr3 environment
cd ~/ASRL/vtr_testing_radar # go to where this repo is located
colcon build --symlink-install
```

wait until it finishes.

Note that whenever you change any code in the vtr3 repo, you need to re-compile and re-install, do this by going to terminal 1 & 2 and re-running the `colcon build ....` command, respectively. Always wait until build process in terminal 1 finishes before running command in terminal 2.

## Run Tests

Go to either of the following

- vtr_testing_radar/src/vtr_testing_radar/script/boreas_testing.sh
- vtr_testing_radar/src/vtr_testing_lidar/script/boreas_testing.sh
  and follow instructions there

Note that any commands specified in these two files should be run inside the container.
