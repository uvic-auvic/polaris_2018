#!/bin/bash

#TODO: Add a -h or --help argument

HOST="localhost"
IP=`hostname -I`
ROS_MASTER_LOC="local"
PORT=11311

# We only want 1 argument, anything more is invalid
if [[ $# -gt 1 ]]; then
    echo "Too many arguments"
    exit 1
elif [[ $# -eq 1 ]]; then # TODO: Replace this with a regex test to test IP validity
    HOST=$1
    ROS_MASTER_LOC="remote"
fi

export ROS_MASTER_URI="http://$HOST:$PORT"
export ROS_IP="${DEFAULT_IP// /}"
export ROS_MASTER_LOC

echo "Using ROS_MASTER_URI $ROS_MASTER_URI"
echo "Using ROS_IP $ROS_IP"

source devel/setup.bash
