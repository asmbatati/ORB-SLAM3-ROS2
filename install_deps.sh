#!/bin/bash
# Use -e to exit on error, but we'll handle specific expected errors
set -e

echo "Installing ORB_SLAM3 dependencies..."

# Update package lists
sudo apt-get update

# Install basic tools and dependencies
sudo apt-get install -y \
    cmake \
    build-essential \
    git \
    unzip \
    pkg-config \
    python3-dev \
    python3-numpy \
    libgl1-mesa-dev \
    libglew-dev \
    libpython3-dev \
    libeigen3-dev \
    apt-transport-https \
    ca-certificates \
    software-properties-common

# Install OpenCV dependencies
sudo apt-get install -y \
    python3-dev \
    python3-numpy \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer1.0-dev \
    libgtk-3-dev

# Install ROS2 related packages
sudo apt-get install -y \
    ros-humble-pcl-ros \
    tmux \
    ros-humble-nav2-common \
    x11-apps \
    nano \
    gdb \
    gdbserver \
    ros-humble-rmw-cyclonedds-cpp

# Build and install Pangolin
echo "Building and installing Pangolin..."
mkdir -p ~/shared_volume/thirdparty
cd ~/shared_volume/thirdparty

# Check if Pangolin directory already exists
if [ ! -d "Pangolin" ]; then
    echo "Cloning Pangolin repository..."
    git clone https://github.com/stevenlovegrove/Pangolin
else
    echo "Pangolin directory already exists, skipping clone..."
fi

cd Pangolin
git checkout v0.9.1
mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-std=c++14 -DCMAKE_INSTALL_PREFIX=/usr/local ..
make -j$(nproc)
sudo make install

# Now build ORB_SLAM3 and its dependencies following the original build.sh script
echo "Building ORB_SLAM3 and its dependencies..."

# Save the current directory
CURRENT_DIR=$(pwd)

# Navigate to ORB_SLAM3 directory
cd ~/shared_volume/orb_slam3_ws/src/ORB-SLAM3-ROS2-Docker/ORB_SLAM3

# Function to run make install and handle the case when install target is missing
run_make_install() {
    # Try to run make install
    if make install 2>/dev/null; then
        echo "make install completed successfully"
    else
        echo "make install target not found, manually copying library files..."
        # Find the library files and create lib directory if it doesn't exist
        mkdir -p ../lib
        # Copy all .so files to the lib directory
        find . -name "*.so" -exec cp {} ../lib/ \;
        echo "Library files copied to lib directory"
    fi
}

echo "Configuring and building Thirdparty/DBoW2 ..."
cd Thirdparty/DBoW2
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-std=c++14
make -j$(nproc)

# Handle the case when there's no install target
run_make_install

echo "Configuring and building Thirdparty/g2o ..."
cd ../../g2o
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-std=c++14
make -j$(nproc)

# Handle the case when there's no install target
run_make_install

echo "Configuring and building Thirdparty/Sophus ..."
cd ../../Sophus
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-std=c++14
make -j$(nproc)

# Sophus is header-only, but try install anyway
run_make_install

cd ../../../

echo "Uncompress vocabulary ..."
cd Vocabulary
if [ -f ORBvoc.txt.tar.gz ]; then
    # Check if it's already been extracted
    if [ ! -f ORBvoc.txt ]; then
        echo "Extracting vocabulary..."
        tar -xf ORBvoc.txt.tar.gz
    else
        echo "Vocabulary already extracted, skipping..."
    fi
fi
cd ..

echo "Configuring and building ORB_SLAM3 ..."
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-std=c++14
make -j$(nproc)

# Return to the original directory
cd "$CURRENT_DIR"

echo "All dependencies and ORB_SLAM3 have been built successfully!"
echo "Now you can build the ROS2 wrapper with: colcon build" 