import tf
import rospy
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from message_filters import TimeSynchronizer, Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import Imu
from mag_common_py_libs.geometry import yaw
from copy import deepcopy

class YawPlotter:
    def __init__(self):
        rospy.init_node("Plotter")
        self.ready_to_plot = False
        self.stand_by_flag = False
        self.fig, self.ax = plt.subplots()
        self.tss = ApproximateTimeSynchronizer([Subscriber("/odom",Odometry), Subscriber("/imu/data_raw_transformed", Imu)],5,0.1)
        self.tss.registerCallback(self.got_velocities)
        self.x = list()
        self.imu_yaw = list()
        self.odom_yaw = list()
        self.clear = False
        self.path_received = False

    def reset(self):
        self.ready_to_plot = False
        self.clear = True
        self.stand_by_flag = True
        self.x = list()
        self.imu_yaw = list()
        self.odom_yaw = list()

    def start(self):
        self.stand_by_flag = False

    def got_velocities(self, odom, imu):
        if self.stand_by_flag:
            return
        self.imu_yaw.append(yaw(imu.orientation))
        self.odom_yaw.append(yaw(odom.pose.pose.orientation))
        self.x.append(len(self.imu_yaw))
        self.ready_to_plot = True

plt.ion()

plot = YawPlotter()
plot.start()

while not rospy.is_shutdown():
    if plot.clear:
        plt.clf()
        plot.clear = False
        plot.start()
    if plot.ready_to_plot:
        x = deepcopy(plot.x)
        y1= deepcopy(plot.imu_yaw)
        y2= deepcopy(plot.odom_yaw)

        if len(x) is len(y1):
            plt.scatter(x, y1, c='r')
        if len(x) is len(y2):
            plt.scatter(x, y2, c='b')
        plot.fig.canvas.draw_idle()
        plt.pause(0.1)
        plot.ready = False

        if len(plot.x) > 500:
            print "RESET"
            plot.reset()
