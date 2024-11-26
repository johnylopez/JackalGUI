import rospy
from nav_msgs.msg import Odometry
import math

# Global variables to store previous position
previous_x = 0.0
previous_y = 0.0

def odom_callback(msg):
    global previous_x, previous_y

    # Get current position from the Odometry message
    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y

    # Calculate distance moved since the last callback
    delta_x = current_x - previous_x
    delta_y = current_y - previous_y
    distance_moved = math.sqrt(delta_x**2 + delta_y**2)

    # Convert to feet
    distance_feet = distance_moved * 3.28084

    # Print the distance moved
    print(f"Distance moved: {distance_feet:.2f} feet")

    # Update the previous position for the next callback
    previous_x = current_x
    previous_y = current_y

def main():
    rospy.init_node('distance_calculator')
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
