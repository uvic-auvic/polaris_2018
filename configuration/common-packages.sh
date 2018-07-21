#!/bin/bash

ROSVERSION=kinetic

sudo apt-get install -y ros-$ROSVERSION-serial
sudo apt-get install -y ros-$ROSVERSION-cv-bridge
sudo apt-get install -y ros-$ROSVERSION-web-video-server
sudo apt-get install -y ros-$ROSVERSION-rosbridge-suite
sudo apt-get install -y fftw3 fftw3-dev pkg-config

