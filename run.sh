#!/bin/bash

# Define the path to your ROS workspace
workspace_path='/home/administrator/workspace_ws'

# Source the ROS setup.bash
source $workspace_path/devel/setup.bash

# Set the ROS master URI
export ROS_MASTER_URI=http://uono01-1:11311

# Run your Python script
python3 /home/administrator/jackalgui/JackalGUI/test.py