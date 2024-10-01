import rospy
from jackal_msgs.msg import Feedback


class ROSJackalMesssagesSubscriber:
    def __init__(self, control_center_callback, msg_topic, node_name):
        self.control_center_callback = control_center_callback
        self.msg_topic = msg_topic
        
        self.control_center_sub = rospy.Subscriber(self.msg_topic, Feedback, self.callback)

    def callback(self, msg):
        self.control_center_callback(msg)