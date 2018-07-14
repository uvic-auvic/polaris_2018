#!/bin/bash

# This file is not a be all end all list of things to install
# It just contains the easiest to install packages which are required
# Anything CUDA related or NVIDIA related should be done using JetPack

sudo apt-get install -y gstreamer1.0-tools libgstreamer1.0-dev libgstreamer-plugins-good1.0-dev libgstreamer-plugins-base1.0-dev
sudo apt-get install -r ros-kinetic-camera-calibration-parsers ros-kinetic-camera-info-manager
./common-packages.sh
pushd ../ros/src
git clone https://github.com/ros-drivers/gscam
sudo apt-get install -y x11vnc
popd

echo "Add '-DGSTREAMER_VERSION_1_x=On' to the EXTRA_CMAKE_FLAGS in the gscam package"
