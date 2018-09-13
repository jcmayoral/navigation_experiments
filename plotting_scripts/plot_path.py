import tf
import rospy
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Path, Odometry
from copy import deepcopy

class Plotter:
    def __init__(self):
        rospy.init_node("Path_Plotter")
        self.fig, self.ax = plt.subplots()
        self.ready = False
        self.x = list()
        self.y = list()
        self.yaw = list()
        self.odom_x = list()
        self.odom_y = list()
        self.odom_yaw = list()
        self.clear = False
        self.path_received = False
        self.listener = tf.TransformListener()
        self.listener.waitForTransform("/odom", "/map", rospy.Time(0),rospy.Duration(1.0))
        rospy.sleep(4)
        rospy.Subscriber("/navigation/move_base_flex/SBPLLatticePlanner/plan", Path,self.pathCB)
        rospy.Subscriber("/odom",Odometry,self.odomCB, queue_size = 1)

    def pathCB(self, msg):
        self.odom_x = list()
        self.odom_y = list()
        self.odom_yaw = list()
        self.path_received = False
        self.clear = True
        self.ready = False

        data = list()
        yaw = list()

        try:
            for p in msg.poses:
                odom_pose = self.listener.transformPose("odom", p)
                explicit_quaternion = [odom_pose.pose.orientation.x, odom_pose.pose.orientation.y, odom_pose.pose.orientation.z, odom_pose.pose.orientation.w]
                euler = tf.transformations.euler_from_quaternion(explicit_quaternion)
                data.append([odom_pose.pose.position.x,odom_pose.pose.position.y])
                yaw.append(euler[2])
                self.x =  [item[0] for item in data]
                self.y =  [item[1] for item in data]
                self.yaw =  [i for i in yaw]
                self.ready = True
                self.path_received = True
            print "HACK WORKED"
        except:
            print "RECALLING"
            self.pathCB(msg)

    def odomCB(self,msg):
        if not self.path_received:
            return
        if self.ready:
            return
        self.ready = False
        self.odom_x.append(msg.pose.pose.position.x)
        self.odom_y.append(msg.pose.pose.position.y)
        explicit_quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        euler = tf.transformations.euler_from_quaternion(explicit_quaternion)
        self.odom_yaw.append(euler[2])
        self.ready = True

plt.ion()

plot = Plotter()
r = 0.3
while not rospy.is_shutdown():
    if plot.ready:
        plot.ready = False
        if plot.clear:
            plt.clf()
            plot.clear = False
        print "PLOTTING"

        x1 = deepcopy(plot.x)
        y1= deepcopy(plot.y)
        yaw1= deepcopy(plot.yaw)
        ox= deepcopy(plot.odom_x)
        oy= deepcopy(plot.odom_y)
        oyaw= deepcopy(plot.odom_yaw)
        plt.scatter(x1, y1, c='r')

        for x,y,w in zip(x1, y1, yaw1):
            plt.arrow(x, y,r*np.cos(w), r*np.sin(w), color='r')
        plt.scatter(ox, oy, c='b')

        for x,y,w in zip(ox, oy, oyaw):
            plt.arrow(x, y,r*np.cos(w), r*np.sin(w), color='b')

        if len(y1) > 0:
            plt.xlim(min(plot.x)-r, max(plot.x)+r)
            plt.xlim(min(plot.x)-r, max(plot.x)+r)
        plot.fig.canvas.draw_idle()
        plt.pause(0.1)
