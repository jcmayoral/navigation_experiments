
import rospy
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Path

class Plotter:
    def __init__(self):
        rospy.init_node("Plotter")
        self.fig, self.ax = plt.subplots()
        rospy.Subscriber("/navigation/move_base_flex/SBPLLatticePlanner/plan", Path,self.pathCB)
        self.ready = False
        self.x = list()
        self.y = list()
        #rospy.spin()

    def pathCB(self, msg):
        self.x = list()
        self.y = list()
        data = list()
        for p in msg.poses:
            data.append([p.pose.position.x,p.pose.position.y])
        self.x =  [item[0] for item in data]
        self.y =  [item[1] for item in data]
        self.ready = True

plt.ion()

plot = Plotter()

while True:
    if plot.ready:
        plt.scatter(plot.x, plot.y)
        plot.fig.canvas.draw_idle()
        plot.ready = False
        plt.pause(0.1)
