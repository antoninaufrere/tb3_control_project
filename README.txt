PROJECT TURTLEBOT3 CONTROL (ROS 2 HUMBLE)

This document explains how to install the dependencies and launch the project, both in simulation and on the real robot.

I) NAVIGATE TO THE PROJECT DIRECTORY

Open a terminal and type:

cd ~/Documents/TB3

Everything that follows must be done from this directory.

II) LOAD THE ROS 2 HUMBLE ENVIRONMENT

In every new terminal window, type:

source /opt/ros/humble/setup.bash

Without this command, ROS tools will not work.

III) INSTALL THE DEPENDENCIES (TO DO ONLY ONCE)

Still inside ~/Documents/TB3:

./install_tb3_deps.sh

If the script is not executable, type the following once:

chmod +x install_tb3_deps.sh
./install_tb3_deps.sh

The script installs the necessary ROS packages as well as the required Python dependencies.

IV) MAKE THE LAUNCH SCRIPTS EXECUTABLE (ONLY ONCE)

chmod +x sim/run_tb3_sim.sh
chmod +x real/run_tb3_real.sh

After this, these commands do not need to be repeated.

V) LAUNCH THE TURTLEBOT3 SIMULATION

5.1. Go to the sim directory:

cd ~/Documents/TB3/sim

(If this is a new terminal window, remember to run:
source /opt/ros/humble/setup.bash )

5.2. Launch the simulation:

./run_tb3_sim.sh

This script launches Gazebo with the TurtleBot3 and the associated ROS nodes.
Keep this terminal window open for the entire duration of the simulation.

VI) LAUNCH THE TURTLEBOT3 IN REAL MODE

6.1 Open a new terminal and type:

(after turning on the router and the TurtleBot)

ssh ubuntu@192.168.0.200

Enter the password.

Then, type:

export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup robot.launch.py
