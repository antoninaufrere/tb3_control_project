#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger

pkill -9 -f gzserver 2>/dev/null
pkill -9 -f gzclient 2>/dev/null
pkill -9 -f robot_state_publisher 2>/dev/null
pkill -9 -f tb3_controller.py 2>/dev/null
sleep 1

echo "Choose Trajectory :"
echo "  1) line"
echo "  2) circle"
echo "  3) eight"
read -r TRAJ_CHOICE

case "$TRAJ_CHOICE" in
  1|"line"*)   TRAJ_ARG="line"   ;;
  2|"circle"*) TRAJ_ARG="circle" ;;
  3|"eight"*)  TRAJ_ARG="eight" ;;
  *) TRAJ_ARG="circle" ;;
esac

echo "Starting TurtleBot3 World"

ros2 launch turtlebot3_gazebo empty_world.launch.py > /tmp/gazebo.log 2>&1 &
SIM_PID=$!


sleep 10


echo "Starting Controller ($TRAJ_ARG)..."
python3 "$SCRIPT_DIR/tb3_controller.py" "$TRAJ_ARG" &
CTRL_PID=$!

# Simulation duration
sleep 60


echo "End of Simulation"
kill -INT "$CTRL_PID" 2>/dev/null

sleep 2

kill -INT "$SIM_PID" 2>/dev/null
sleep 2
pkill -f gzclient
pkill -f gzserver

#PLOT
if [ -f "$SCRIPT_DIR/plot_tb3_traj.py" ]; then
    echo "Plot"
    python3 "$SCRIPT_DIR/plot_tb3_traj.py"
fi
