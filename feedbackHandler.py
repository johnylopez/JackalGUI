import rospy
from jackal_msgs.msg import Feedback
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import Bool


class ROSJackalMesssagesSubscriber:
    def __init__(self, control_center_callback, msg_topic, node_name):
        self.control_center_callback = control_center_callback
        self.msg_topic = msg_topic
        
        self.control_center_sub = rospy.Subscriber(self.msg_topic, Feedback, self.callback)

    def callback(self, msg):
        self.control_center_callback(msg)


class ROSJackalDiagnosticMesssagesSubscriber:
    def __init__(self, control_center_callback, msg_topic):
        self.control_center_callback = control_center_callback
        self.msg_topic = msg_topic
        
        self.control_center_sub = rospy.Subscriber(self.msg_topic, DiagnosticArray, self.callback)

    def callback(self, msg):
        self.control_center_callback(msg)


class ROSJackalWifiConnectedSubscriber:
    def __init__(self, wifi_callback, msg_topic):
        self.wifi_callback = wifi_callback
        self.msg_topic = msg_topic
        
        self.sub = rospy.Subscriber(self.msg_topic, Bool, self.callback)

    def callback(self, msg):
        self.wifi_callback(msg)