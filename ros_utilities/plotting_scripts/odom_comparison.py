import tf
import rospy
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from copy import deepcopy
import threading

class OdomPlotter:
    def __init__(self, topic_name):
        #self.fig, self.ax = plt.subplots()
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.is_started = False
        self.clear = False
        self.lock = threading.Lock()
        self.listener = tf.TransformListener()
        rospy.Subscriber(topic_name,Odometry,self.odomCB, queue_size = 1)
        rospy.loginfo("PLOT READY")

    def odomCB(self,msg):
        self.is_started = True
        odom_pose = PoseStamped()
        odom_pose.header = msg.header
        odom_pose.pose = msg.pose.pose
        explicit_quaternion = [odom_pose.pose.orientation.x, odom_pose.pose.orientation.y, \
                               odom_pose.pose.orientation.z, odom_pose.pose.orientation.w]
        euler = tf.transformations.euler_from_quaternion(explicit_quaternion)
        self.lock.acquire()
        self.x = odom_pose.pose.position.x
        self.y = odom_pose.pose.position.y
        self.yaw = euler[2]
        self.lock.release()

def reset_cb(event):
    plt.clf()

rospy.init_node("odom_comparison_plot")
plt.ion()
fig, ax = plt.subplots()
#plt.subplots_adjust(bottom=0.2)
plot = OdomPlotter(topic_name="/odom")
ekf_plot = OdomPlotter(topic_name="/fake_odom")
r = 1.5
plt.connect('button_press_event', reset_cb)

while not rospy.is_shutdown():
    if plot.is_started or ekf_plot.is_started:
        ax.legend()

        if plot.is_started:
            plot.lock.acquire()
            x1 = deepcopy(plot.x)
            y1= deepcopy(plot.y)
            yaw1= deepcopy(plot.yaw)
            plot.lock.release()
            plt.scatter(x1, y1, c='r')
            #for x,y,w in zip(x1, y1, yaw1):
            plt.arrow(x1, y1,r*np.cos(yaw1), r*np.sin(yaw1), color='r',  label='odom', head_width=0.025)

        if ekf_plot.is_started:
            ekf_plot.lock.acquire()
            ox= deepcopy(ekf_plot.x)
            oy= deepcopy(ekf_plot.y)
            oyaw= deepcopy(ekf_plot.yaw)
            ekf_plot.lock.release()
            plt.scatter(ox, oy, c='b')
            plt.arrow(ox, oy,r*np.cos(oyaw), r*np.sin(oyaw), color='b',  label='fake_odom', head_width=0.025)

        fig.canvas.draw_idle()
        #plt.draw()
        plt.pause(0.1)
