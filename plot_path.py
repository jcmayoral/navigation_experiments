import tf
import rospy
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Path, Odometry

class Plotter:
    def __init__(self):
        rospy.init_node("Plotter")
        self.fig, self.ax = plt.subplots()
        rospy.Subscriber("/navigation/move_base_flex/SBPLLatticePlanner/plan", Path,self.pathCB)
        rospy.Subscriber("/odom",Odometry,self.odomCB)
        self.listener = tf.TransformListener()
        self.ready = False
        self.x = list()
        self.y = list()
        self.odom_x = list()
        self.odom_y = list()
        self.clear = False
        self.busy = False
        #rospy.spin()

    def pathCB(self, msg):
        self.clear = True
        self.listener.waitForTransform("map", "odom", rospy.Time.now(),rospy.Duration(.1))
        self.x = list()
        self.y = list()
        self.odom_x = list()
        self.odom_y = list()

        data = list()
        for p in msg.poses:
            odom_pose = self.listener.transformPose("odom", p)
            data.append([odom_pose.pose.position.x,odom_pose.pose.position.y])
        self.x =  [item[0] for item in data]
        self.y =  [item[1] for item in data]
        self.ready = True

    def odomCB(self,msg):
        self.odom_x.append(msg.pose.pose.position.x)
        self.odom_y.append(msg.pose.pose.position.y)
        self.ready = True

plt.ion()

plot = Plotter()

while not rospy.is_shutdown():
    if plot.ready:
        if plot.clear:
            plt.clf()
            plot.clear = False
        plt.scatter(plot.x, plot.y, c='r')
        plt.scatter(plot.odom_x, plot.odom_y, c='b')
        plot.fig.canvas.draw_idle()
        plt.pause(0.1)
        plot.ready = False
