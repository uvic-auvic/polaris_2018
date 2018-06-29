# This script installs the full desktop suite of ROS (Robotic Operating System) on Ubuntu 14.04 based systems
# It should be run as root in order to run uninhibited

ROSVERSION="kinetic"

# Add ROS Package to list with keys
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

# Install the core ROS programs
sudo apt-get update
sudo apt-get install -y ros-$ROSVERSION-desktop-full

# Initialize ROSDEP
sudo rosdep init
rosdep update

# Setup environment. Comments out if you don't use bash
echo "source /opt/ros/$ROSVERSION/setup.bash" >> ~/.bashrc
source ~/.bashrc
mkdir -p ~/rosbag

# Install rosinstall and other build tools
sudo apt-get install -y python-rosinstall python-rosinstall-generator python-wstool build-essential python-pip

# Install all the ROS packages
./common-packages.sh
