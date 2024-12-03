#!/bin/bash

# Make this directory into a ros workspace
mkdir -p /src
cd /src
catkin_init_workspace

# Download the dataset if it doesn't exist
if [ ! -f bonirob_2016-04-20-15-43-50_10.bag ]; then
    echo "Downloading dataset..."
    mkdir -p /data/bags
    curl -o /data/bags/bonirob_2016-04-20-15-43-50_10.bag https://www.ipb.uni-bonn.de/datasets_IJRR2017/rosbags/160420/bonirob_2016-04-20-15-43-50_10.bag
fi

# Keep the container running
tail -f /dev/null