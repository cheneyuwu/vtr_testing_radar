# Installation

## Recommend to remove your current local vtr3 repo completely and re-clone it from github!

# Follow the GitHub installation instructions: https://github.com/utiasASRL/vtr3/wiki/Installation-Guide
# with the following changes:

#   - after downloading the vtr3 repo, switch to `main_radar` branch before updating submodules
      cd ${VTRSRC}
      git clone git@github.com:utiasASRL/vtr3.git .
      git checkout main_radar  # !!!
      git submodule update --init --remote

#   - for radar there's no need to change anything in gpusurf, since we won't use it anyway

#   - cuda is also optional to install, we won't use it anyway

#   - we will use the apt version of OpenCV, so DO NOT install it from source. Skip `Install OpenCV` section completely. If you have installed, remove it.

#   - install ROS2 Galactic from apt, do not build it from source

#   - also, in the `Install dependencies from ROS2 packages` section, use the apt install option for all packages, do not build them from source

#   - skip the `Build and install driver and robot description packages` section as we are only evaluating the localization mapping pipeline

#   - Build and install VT&R3, use the following command instead, and do not build the GUI (not necessary)
      source /opt/ros/galactic/setup.bash  # source the ROS environment
      colcon build --symlink-install --packages-up-to vtr_radar  # only build up to the vtr_radar package, we don't need the other packages

#   - After building vtr3, we need to build the vtr_testing_radar package, using the following:
      source ${VTRSRC}/main/install/setup.bash  # source the vtr3 environment
      cd ~/ASRL/vtr_testing_radar  # go to where this repo is located
      colcon build --symlink-install

