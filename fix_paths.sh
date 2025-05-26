#!/bin/bash
set -e

# Update the path in FindORB_SLAM3.cmake
echo "Fixing ORB_SLAM3 path in FindORB_SLAM3.cmake..."

# The path to the FindORB_SLAM3.cmake file
CMAKE_FILE=~/shared_volume/orb_slam3_ws/src/ORB-SLAM3-ROS2-Docker/orb_slam3_ros2_wrapper/CMakeModules/FindORB_SLAM3.cmake

# Check if the file exists
if [ ! -f "$CMAKE_FILE" ]; then
    echo "Error: FindORB_SLAM3.cmake file not found at $CMAKE_FILE"
    exit 1
fi

# Create a backup of the original file
cp "$CMAKE_FILE" "${CMAKE_FILE}.bak"
echo "Created backup at ${CMAKE_FILE}.bak"

# Get the absolute path to ORB_SLAM3
ORB_SLAM3_PATH=$(realpath ~/shared_volume/orb_slam3_ws/src/ORB-SLAM3-ROS2-Docker/ORB_SLAM3)

# Update the path from /home/orb/ORB_SLAM3 to the actual path in the shared volume
sed -i "s|set(ORB_SLAM3_ROOT_DIR \"/home/orb/ORB_SLAM3\")|set(ORB_SLAM3_ROOT_DIR \"$ORB_SLAM3_PATH\")|g" "$CMAKE_FILE"

echo "Path fixed. Now you can run the install_deps.sh script and then build the project." 