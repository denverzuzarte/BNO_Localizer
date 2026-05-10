#!/bin/bash
source "$(dirname "$0")/install/setup.bash"

for topic in /imu1/data/gyro /imu1/data/lin_acc /imu1/data/rot_vec; do
    echo -n "$topic: "
    timeout 5 ros2 topic hz --window 40 "$topic" 2>&1 | grep "average rate" | tail -1 | grep -oP '[0-9]+\.[0-9]+' | head -1
done
