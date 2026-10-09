#!/bin/bash
set -e

echo "Installing system dependencies..."
echo '12345' | sudo -S apt-get update
echo '12345' | sudo -S apt-get install -y python3-rpi.gpio cmake pkg-config git build-essential

echo "Installing YDLidar SDK..."
cd ~
if [ ! -d "YDLidar-SDK" ]; then
    git clone https://github.com/YDLIDAR/YDLidar-SDK.git
    cd YDLidar-SDK
    mkdir -p build
    cd build
    cmake ..
    make
    echo '12345' | sudo -S make install
else
    echo "YDLidar SDK already cloned."
fi

echo "Cloning YDLidar ROS 2 Driver..."
cd ~/robot_ws/src
if [ ! -d "ydlidar_ros2_driver" ]; then
    git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git -b humble
else
    echo "ydlidar_ros2_driver already cloned."
fi

echo "Done installing dependencies."
