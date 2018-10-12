import rospy
import numpy as np
from message_filters import Subscriber, ApproximateTimeSynchronizer
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from copy import deepcopy

class WheelSyncronizer:
    def __init__(self):
        rospy.init_node("wheels_sync")
        self.R = 0.5
        self.L = 1
        self.odom_vel_pub = rospy.Publisher("recomputed_odom_vel", Twist, queue_size = 1)
        self.diff_pub = rospy.Publisher("magazino_fdd/wheel_diff", Float64, queue_size = 1)
        self.tss = ApproximateTimeSynchronizer([Subscriber("/left_speed",Float64), Subscriber("/right_speed", Float64)],5,0.1, allow_headerless=True)
        self.tss.registerCallback(self.got_velocities)

    def got_velocities(self, left_wheel, right_wheel):
        twist_msg = Twist()
        self.diff_pub.publish(Float64(data = left_wheel.data - right_wheel.data))
        twist_msg.linear.x = self.R *(left_wheel.data + right_wheel.data)/2
        twist_msg.angular.z = self.R/self.L * (right_wheel.data - left_wheel.data)
        self.odom_vel_pub.publish(twist_msg)


"""
input v,w

#CYCLE
-x- = v cos phy
-y- = v sin phy
-phy- = w

#DIFF DRIVE
-x- = R/2 (vr+vl) cos(phy)
-y- = R/2 (vr+vl) sin(phy)
w = R/L (vr-vl)

#Substitution
v = R/2 (vr+vl)
w = R/L (vr-vl)

vr = 2v/R - vl
vl = vr - Lw/R

vr = (2v + Lw)/2R
vl = (2v - Lw)/2R

#Inverting
2R vr - Lw = 2v
2v - 2R vl =  Lw

2R (vr + vl) = 4v
2R (vr - vl) = 2Lw

"""
