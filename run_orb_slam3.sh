#!/bin/bash
set -e

# Get the actual ORB_SLAM3 library path
ORB_SLAM3_PATH=~/shared_volume/orb_slam3_ws/src/ORB-SLAM3-ROS2-Docker/ORB_SLAM3
ORB_SLAM3_LIB_PATH=$ORB_SLAM3_PATH/lib
THIRD_PARTY_PATH=$ORB_SLAM3_PATH/Thirdparty

# Ensure the libraries exist
echo "Checking for ORB_SLAM3 libraries..."

if [ ! -f "$ORB_SLAM3_LIB_PATH/libORB_SLAM3.so" ]; then
    echo "ERROR: libORB_SLAM3.so not found at $ORB_SLAM3_LIB_PATH"
    echo "Make sure you've built ORB_SLAM3 successfully."
    exit 1
fi

if [ ! -f "$THIRD_PARTY_PATH/DBoW2/lib/libDBoW2.so" ]; then
    echo "ERROR: libDBoW2.so not found at $THIRD_PARTY_PATH/DBoW2/lib"
    echo "Make sure you've built DBoW2 successfully."
    exit 1
fi

if [ ! -f "$THIRD_PARTY_PATH/g2o/lib/libg2o.so" ]; then
    echo "ERROR: libg2o.so not found at $THIRD_PARTY_PATH/g2o/lib"
    echo "Make sure you've built g2o successfully."
    exit 1
fi

# Create a vocabulary path variable
VOCABULARY_PATH=$ORB_SLAM3_PATH/Vocabulary/ORBvoc.txt

# Setup environment variables
echo "Setting up environment variables..."
export LD_LIBRARY_PATH=$ORB_SLAM3_LIB_PATH:$THIRD_PARTY_PATH/DBoW2/lib:$THIRD_PARTY_PATH/g2o/lib:$LD_LIBRARY_PATH

# Source ROS2 setup
source /opt/ros/humble/setup.bash
source ~/shared_volume/orb_slam3_ws/install/setup.bash

# Print status
echo "LD_LIBRARY_PATH set to: $LD_LIBRARY_PATH"
echo "-------------------------------------------------------------------------------------"
echo "NOTE: Visualization has been disabled in the config file to avoid X Window System errors."
echo "ORB-SLAM3 will run in headless mode and publish data via ROS2 topics instead."
echo "You can visualize the data using RViz or other ROS2 tools."
echo "-------------------------------------------------------------------------------------"
echo "Launching ORB-SLAM3 ROS2 Wrapper..."

# Launch the node using ros2 launch
ros2 launch orb_slam3_ros2_wrapper unirobot.launch.py

# If you want to run the node directly, uncomment this and comment the ros2 launch command above
# ~/shared_volume/orb_slam3_ws/install/orb_slam3_ros2_wrapper/lib/orb_slam3_ros2_wrapper/rgbd $VOCABULARY_PATH ~/shared_volume/orb_slam3_ws/src/ORB-SLAM3-ROS2-Docker/orb_slam3_ros2_wrapper/params/gazebo_rgbd.yaml 