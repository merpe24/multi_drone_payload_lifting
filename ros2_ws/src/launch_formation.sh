#!/bin/bash
echo "Launching Leader..."
python3 offboard_control.py &

echo "Launching Follower..."
python3 offboard_control.py --ros-args -r __ns:=/px4_1 &

# Wait for both background processes to finish
wait