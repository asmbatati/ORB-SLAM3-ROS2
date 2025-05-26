#!/bin/bash
set -e

# Get the actual ORB_SLAM3 library path
ORB_SLAM3_PATH=~/shared_volume/orb_slam3_ws/src/ORB-SLAM3-ROS2/ORB_SLAM3
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

# Create symbolic links in /usr/local/lib
echo "Creating symbolic links to libraries in /usr/local/lib..."
sudo ln -sf $ORB_SLAM3_LIB_PATH/libORB_SLAM3.so /usr/local/lib/
sudo ln -sf $THIRD_PARTY_PATH/DBoW2/lib/libDBoW2.so /usr/local/lib/
sudo ln -sf $THIRD_PARTY_PATH/g2o/lib/libg2o.so /usr/local/lib/

# Update the library cache
sudo ldconfig

echo "Libraries installed successfully."
echo "You can now run ORB-SLAM3 without setting LD_LIBRARY_PATH manually." 