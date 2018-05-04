#!/bin/bash

# This file is not a be all end all list of things to install
# It just contains the easiest to install packages which are required
# Anything CUDA related or NVIDIA related should be done using JetPack

dev libgstreamer-plugins-good1.0-dev
cd ../ros/src
git clone https://github.com/ros-drivers/gscam

./common-packages.sh

echo "Add '-DGSTREAMER_VERSION_1_x=0n' to the EXTRA_CMAKE_FLAGS in the gscam package"
