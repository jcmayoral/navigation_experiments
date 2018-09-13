import rospy
import tf
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Path, Odometry

class Plotter:
    def __init__(self):
        rospy.init_node("Plotter")
        self.fig, self.ax = plt.subplots()
        self.ready = False
        self.x = list()
        self.y = list()
        self.yaw = list()
        self.local_x = list()
        self.local_y = list()
        self.local_yaw = list()
        self.clear = False
        rospy.Subscriber("/navigation/move_base_flex/SBPLLatticePlanner/plan", Path, self.pathCB, "GLOBAL")
        rospy.Subscriber("/navigation/move_base_flex/OrientedDWAPlanner/local_plan", Path, self.pathCB, "LOCAL")

    def pathCB(self, msg, path_type):
        x = list()
        y = list()
        yaw = list()

        data = list()
        yaw = list()

        for p in msg.poses:
            explicit_quaternion = [p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w]
            euler = tf.transformations.euler_from_quaternion(explicit_quaternion)
            data.append([p.pose.position.x, p.pose.position.y])
            yaw.append(euler[2])

        if path_type is "GLOBAL":
            self.x =  [item[0] for item in data]
            self.y =  [item[1] for item in data]
            self.yaw =  [i for i in yaw]
            self.ready = False
        else:
            self.local_x =  [item[0] for item in data]
            self.local_y =  [item[1] for item in data]
            self.local_yaw =  [i for i in yaw]
            self.ready = True
            self.clear = True

plt.ion()

plot = Plotter()
r = 1.0
while not rospy.is_shutdown():
    if plot.ready:
        if plot.clear:
            plt.clf()
            plot.clear = False
        plt.scatter(plot.x, plot.y, c='r')
        for x,y,w in zip(plot.x, plot.y, plot.yaw):
            plt.arrow(x, y,r*np.cos(w), r*np.sin(w), color='r')
        plt.scatter(plot.local_x, plot.local_y, c='b')
        for x,y,w in zip(plot.local_x, plot.local_y, plot.local_yaw):
            plt.arrow(x, y,r*np.cos(w), r*np.sin(w), color='b')

        if len(plot.y) > 0:
            plt.ylim(min(plot.y)-r, max(plot.y)+r)
            plt.xlim(min(plot.x)-r, max(plot.x)+r)
        plot.fig.canvas.draw_idle()
        plt.pause(0.1)
        plot.ready = False
