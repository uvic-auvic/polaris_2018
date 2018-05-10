# Polaris Source Code

All the code related to the operation of the polaris 2018 AUVIC submarine

## Setting up your enviroment

1. Create a fork of the polaris repo and clone it in your ubuntu workspace `git clone https://github.com/yourusername/polaris.git`

2. Navigate to the configuration folder by `cd configurtion` and run the setup file with `./setup.sh`

3. After script has finished running, close the terminal window and open a new one. To ensure ROS has properly installed run the command `roscore`. If successful roscore will launch.

4. if successful navigate to the `ros` directory and run Run the command `catkin_make`

5. Your catkin workspace is now set up inside your local git fork.

6. Add the remote upstream when `git remote add upstream https://github.com/uvic-auvic/polaris.git`
