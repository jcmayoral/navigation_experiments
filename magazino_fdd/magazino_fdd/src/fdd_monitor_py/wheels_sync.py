import rospy
import numpy as np
from message_filters import Subscriber, ApproximateTimeSynchronizer
from std_msgs.msg import Float64
from copy import deepcopy

class WheelSyncronizer:
    def __init__(self):
        rospy.init_node("wheels_sync")
        self.diff_pub = rospy.Publisher("magazino_fdd/wheel_diff", Float64, queue_size = 1)
        self.tss = ApproximateTimeSynchronizer([Subscriber("/left_speed",Float64), Subscriber("/right_speed", Float64)],5,0.1, allow_headerless=True)
        self.tss.registerCallback(self.got_velocities)

    def got_velocities(self, left_wheel, right_wheel):
        self.diff_pub.publish(Float64(data = left_wheel.data - right_wheel.data))
