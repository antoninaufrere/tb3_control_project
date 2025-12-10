#!/bin/bash


SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# CONFIG ROS
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger


DURATION=60


echo "Choose trajectory :"
echo "  1) line"
echo "  2) circle"
echo "  3) eight"
read -r TRAJ_CHOICE

case "$TRAJ_CHOICE" in
  1|"line"*)   TRAJ_ARG="line" ;;
  2|"circle"*) TRAJ_ARG="circle" ;;
  3|"eight"*)  TRAJ_ARG="eight" ;;
  *)           TRAJ_ARG="circle" ;;
esac



echo "Starting Controller"

python3 "$SCRIPT_DIR/tb3_controller_in_real.py" "$TRAJ_ARG" &
CTRL_PID=$!

echo "Executing during ${DURATION}s..."
sleep "$DURATION"


echo "Stopping controller"
kill -INT "$CTRL_PID" 2>/dev/null
wait "$CTRL_PID" 2>/dev/null


ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" > /dev/null 2>&1

sleep 1

# Plot
echo "Plot"
if [ -f "$SCRIPT_DIR/plot_tb3_traj.py" ]; then
    python3 "$SCRIPT_DIR/plot_tb3_traj.py"
fi
