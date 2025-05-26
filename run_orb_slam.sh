#!/bin/bash
set -e

# Define color codes for better visibility
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${YELLOW}Starting ORB-SLAM3 for dem_stereo simulation...${NC}"

# Define paths
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
ORB_SLAM3_PATH="$SCRIPT_DIR/ORB_SLAM3"
ORB_SLAM3_LIB_PATH="$ORB_SLAM3_PATH/lib"
THIRD_PARTY_PATH="$ORB_SLAM3_PATH/Thirdparty"
VOCAB_FILE="$ORB_SLAM3_PATH/Vocabulary/ORBvoc.txt"
CONFIG_FILE="$SCRIPT_DIR/orb_slam3_ros2_wrapper/params/gazebo_rgbd.yaml"

# Ensure the libraries exist
echo -e "${YELLOW}Checking for ORB_SLAM3 libraries...${NC}"

if [ ! -f "$ORB_SLAM3_LIB_PATH/libORB_SLAM3.so" ]; then
    echo -e "${RED}ERROR: libORB_SLAM3.so not found at $ORB_SLAM3_LIB_PATH${NC}"
    echo -e "${RED}Make sure you've built ORB_SLAM3 successfully.${NC}"
    exit 1
fi

if [ ! -f "$THIRD_PARTY_PATH/DBoW2/lib/libDBoW2.so" ]; then
    echo -e "${RED}ERROR: libDBoW2.so not found at $THIRD_PARTY_PATH/DBoW2/lib${NC}"
    echo -e "${RED}Make sure you've built DBoW2 successfully.${NC}"
    exit 1
fi

if [ ! -f "$THIRD_PARTY_PATH/g2o/lib/libg2o.so" ]; then
    echo -e "${RED}ERROR: libg2o.so not found at $THIRD_PARTY_PATH/g2o/lib${NC}"
    echo -e "${RED}Make sure you've built g2o successfully.${NC}"
    exit 1
fi

# Check vocabulary and config files
echo -e "${YELLOW}Checking for vocabulary and configuration files...${NC}"
if [ ! -f "$VOCAB_FILE" ]; then
    echo -e "${RED}ERROR: Vocabulary file not found at $VOCAB_FILE${NC}"
    exit 1
fi

if [ ! -f "$CONFIG_FILE" ]; then
    echo -e "${RED}ERROR: Configuration file not found at $CONFIG_FILE${NC}"
    exit 1
fi

# Check if the simulation is running by checking for required topics
echo -e "${YELLOW}Checking if the dem_stereo.launch.py simulation is running...${NC}"

# Get list of topics and check for required ones
TOPICS=$(ros2 topic list)

if ! echo "$TOPICS" | grep -q "/target/front_stereo/left_cam/image_raw"; then
    echo -e "${RED}ERROR: Required topic '/target/front_stereo/left_cam/image_raw' not found.${NC}"
    echo -e "${RED}Make sure the dem_stereo.launch.py simulation is running first.${NC}"
    echo -e "${RED}Run: ros2 launch gps_denied_navigation_sim dem_stereo.launch.py${NC}"
    exit 1
fi

# Setup environment variables
echo -e "${YELLOW}Setting up environment variables...${NC}"
export LD_LIBRARY_PATH=$ORB_SLAM3_LIB_PATH:$THIRD_PARTY_PATH/DBoW2/lib:$THIRD_PARTY_PATH/g2o/lib:$LD_LIBRARY_PATH

# Source ROS2 setup
source /opt/ros/humble/setup.bash
source ~/shared_volume/orb_slam3_ws/install/setup.bash

# Print status
echo -e "${GREEN}LD_LIBRARY_PATH set to: $LD_LIBRARY_PATH${NC}"
echo -e "${GREEN}-------------------------------------------------------------------------------------${NC}"
echo -e "${GREEN}NOTE: Visualization has been disabled in the config file to avoid X Window System errors.${NC}"
echo -e "${GREEN}ORB-SLAM3 will run in headless mode and publish data via ROS2 topics instead.${NC}"
echo -e "${GREEN}You can visualize the data using RViz or other ROS2 tools.${NC}"
echo -e "${GREEN}-------------------------------------------------------------------------------------${NC}"
echo -e "${GREEN}Launching ORB-SLAM3 RGBD ROS2 Wrapper with dem_sim.launch.py...${NC}"

# Launch with the dem_sim.launch.py file
ros2 launch orb_slam3_ros2_wrapper dem_sim.launch.py robot_namespace:=target use_sim_time:=true

# Note for users
echo -e "${YELLOW}ORB-SLAM3 has been launched and is now processing stereo data in RGBD mode.${NC}"
echo -e "${YELLOW}You can view the published topics with: ros2 topic list | grep slam${NC}" 