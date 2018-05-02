#!/bin/bash

# This file is not a be all end all list of things to install
# It just contains the easiest to install packages which are required
# Anything CUDA related or NVIDIA related should be done using JetPack

ROSVERSION=kinetic

sudo apt-get install -y ros-$ROSVERSION-usb-cam
sudo apt-get install gstreamer1.0-tools libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev
cd ../ros/src
git clone https://github.com/ros-drivers/gscam

echo "Add '-DGSTREAMER_VERSION_1_x=0n' to the EXTRA_CMAKE_FLAGS in the gscam package"
