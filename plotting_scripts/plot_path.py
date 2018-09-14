import tf
import rospy
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from copy import deepcopy
import threading

class Plotter:
    def __init__(self):
        rospy.init_node("Path_Plotter")
        self.fig, self.ax = plt.subplots()
        self.x = list()
        self.y = list()
        self.yaw = list()
        self.odom_x = list()
        self.odom_y = list()
        self.odom_yaw = list()
        self.clear = False
        self.lock = threading.Lock()
        self.listener = tf.TransformListener()
        rospy.Subscriber("/navigation/move_base_flex/SBPLLatticePlanner/plan", Path,self.pathCB)
        rospy.Subscriber("/odom",Odometry,self.odomCB, queue_size = 1)
        rospy.sleep(4)
        rospy.loginfo("PLOT READY")

    def pathCB(self, msg):
        self.lock.acquire()
        self.odom_x = list()
        self.odom_y = list()
        self.odom_yaw = list()
        self.clear = True

        data = list()
        yaw = list()

        for p in msg.poses:
            explicit_quaternion = [p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w]
            euler = tf.transformations.euler_from_quaternion(explicit_quaternion)
            data.append([p.pose.position.x,p.pose.position.y])
            yaw.append(euler[2])

        self.x =  [item[0] for item in data]
        self.y =  [item[1] for item in data]
        self.yaw =  [i for i in yaw]

        self.lock.release()

    def odomCB(self,msg):
        self.lock.acquire()

        p = PoseStamped()
        p.header = msg.header
        p.header.stamp = rospy.Time.now()
        p.pose = msg.pose.pose

        self.listener.waitForTransform("map", "odom", rospy.Time.now(),rospy.Duration(1.0))
        odom_pose = self.listener.transformPose("map", p)
        explicit_quaternion = [odom_pose.pose.orientation.x, odom_pose.pose.orientation.y, \
        odom_pose.pose.orientation.z, odom_pose.pose.orientation.w]
        euler = tf.transformations.euler_from_quaternion(explicit_quaternion)
        self.odom_x.append(odom_pose.pose.position.x)
        self.odom_y.append(odom_pose.pose.position.y)
        self.odom_yaw.append(euler[2])
        self.lock.release()

plt.ion()

plot = Plotter()
r = 0.3
while not rospy.is_shutdown():
    plot.lock.acquire()

    if plot.clear:
        plt.clf()
        plot.clear = False

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
    plot.lock.release()
    plt.pause(0.1)
