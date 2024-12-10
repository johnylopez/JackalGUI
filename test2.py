# import rospy
# from nav_msgs.msg import Odometry
# import math

# # Global variables to store previous position
# previous_x = 0.0
# previous_y = 0.0

# def odom_callback(msg):
#     global previous_x, previous_y

#     # Get current position from the Odometry message
#     current_x = msg.pose.pose.position.x
#     current_y = msg.pose.pose.position.y

#     # Calculate distance moved since the last callback
#     delta_x = current_x - previous_x
#     delta_y = current_y - previous_y
#     distance_moved = math.sqrt(delta_x**2 + delta_y**2)

#     # Convert to feet
#     distance_feet = distance_moved * 3.28084

#     # Print the distance moved
#     print(f"Distance moved: {distance_feet:.2f} feet")

#     # Update the previous position for the next callback
#     previous_x = current_x
#     previous_y = current_y

# def main():
#     rospy.init_node('distance_calculator')
#     rospy.Subscriber("/odom", Odometry, odom_callback)
#     rospy.spin()

# if __name__ == '__main__':
#     main()


# from collections import defaultdict

# # Given dictionary of deficiencies at different positions
# my_dict = {
#     '0.0': 'crack, fracture',
#     '-0.0': 'crack, fracture',
#     '0.1': 'crack, fracture',
#     '0.2': 'crack',
#     '0.3': 'joints'
# }

# # Function to group positions into 5ft intervals
# def group_by_5ft(position):
#     # Convert the position to a float and calculate the 5ft interval
#     position = float(position)
#     return int(position // 5) * 5  # Returns the lower bound of the interval

# # Create a dictionary to store deficiency counts by 5ft intervals
# deficiency_counts = defaultdict(lambda: defaultdict(int))

# # Group the deficiencies by 5ft interval and count their occurrences
# for position, deficiency in my_dict.items():
#     interval = group_by_5ft(position)  # Get the 5ft interval
#     deficiency_counts[interval][deficiency] += 1  # Count the deficiencies in that interval

# # Find the most recurrent deficiency for each interval
# most_recurrent = {}

# for interval, deficiencies in deficiency_counts.items():
#     # Find the deficiency with the highest count in the current interval
#     most_recurrent[interval] = max(deficiencies, key=deficiencies.get)

# # Print the results
# print("Most recurrent deficiencies for each 5ft interval:")
# for interval, deficiency in most_recurrent.items():
#     print(f"Interval {interval}ft - {interval+4}ft: {deficiency}")


import numpy as np
from collections import defaultdict

# Function to group positions into 5ft intervals
def group_by_5ft(position):
    position = float(position)
    return int(position // 5) * 5  # Returns the lower bound of the interval

# Assuming self.deficiencies_log is structured like:
# self.deficiencies_log = { "3.0": {"detections": ['deficiency_A'], "image": numpy_array_1}, 
#                           "7.1": {"detections": ['deficiency_B'], "image": numpy_array_2},
#                           ... }

# Initialize the dictionary to store deficiency counts by 5ft intervals
deficiency_counts = defaultdict(lambda: defaultdict(int))

# Group the deficiencies by 5ft interval and count their occurrences
for position_str, value in self.deficiencies_log.items():
    position = float(position_str)  # Convert the key (position) back to a float
    detections = value["detections"]  # Get the detections for this position
    interval = group_by_5ft(position)  # Get the 5ft interval
    
    # Count deficiencies in this interval
    for detection in detections:
        deficiency_counts[interval][detection] += 1

# Find the most recurrent deficiency for each interval
most_recurrent = {}

for interval, deficiencies in deficiency_counts.items():
    most_recurrent[interval] = max(deficiencies, key=deficiencies.get)

# Print the results
print("Most recurrent deficiencies for each 5ft interval:")
for interval, deficiency in most_recurrent.items():
    print(f"Interval {interval}ft - {interval+4}ft: {deficiency}")
