import subprocess
import os

# Define the path to your ROS workspace
workspace_path = '/home/administrator/workspace_ws'

# Source the ROS setup.bash
subprocess.call(['bash', '-c', f'source {workspace_path}/devel/setup.bash && rostopic list'])