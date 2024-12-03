#!/bin/bash

# Source ROS environment
source /opt/ros/rolling/setup.bash

# Make this directory into a ros workspace
cd ros
pipx ensurepath
source ~/.bashrc


# Download datasets listed in data_files.txt if they don't exist¨
for filename in $(cat data_files.txt); do
    if [ ! -f "/ros/data/bags/$filename" ]; then
        echo "Downloading dataset $filename..."
        mkdir -p /ros/data/bags
        curl -o "/ros/data/bags/$filename" "https://www.ipb.uni-bonn.de/datasets_IJRR2017/rosbags/160420/$filename"
        /root/.local/bin/rosbags-convert --src "/ros/data/bags/$filename" --dst "/ros/data/bags/${filename}_ros2"
    fi
done 


# Keep the container running
tail -f /dev/null