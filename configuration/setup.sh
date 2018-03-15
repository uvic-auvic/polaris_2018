# This script installs the full desktop suite of ROS (Robotic Operating System) on Ubuntu 14.04 based systems
# It should be run as root in order to run uninhibited

ROS_VERSION="kinetic"

# Add ROS Package to list with keys
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

# Install the core ROS programs
sudo apt-get update
sudo apt-get install -y ros-$ROS_VERSION-desktop-full

# Install all the 3rd party packages
PACK_FILENAME="ros-packages.list"
while read -r line
do
    PACKAGES="ros-$ROS_VERSION-$line $PACKAGES"
done < $PACK_FILENAME
sudo apt-get install -y $PACKAGES

# Initialize ROSDEP
sudo rosdep init
rosdep update

# Setup environment. Comments out if you don't use bash
echo "source /opt/ros/$ROS_VERSION/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install rosinstall and other build tools
sudo apt-get install -y python-rosinstall python-rosinstall-generator python-wstool build-essential
