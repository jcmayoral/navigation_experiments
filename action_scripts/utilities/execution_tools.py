import tf
import rospy
import numpy as np
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from message_filters import TimeSynchronizer, Subscriber, ApproximateTimeSynchronizer
from mag_common_py_libs.geometry import yaw


class PathConstructor:
    def __init__(self, path_frame = "odom"):
        rospy.Subscriber("/odom",Odometry,self.odomCB)
        self.listener = tf.TransformListener()
        self.constructed_path = Path()
        self.constructed_path.header.frame_id = path_frame
        self.x = list()
        self.y = list()
        self.yaw = list()
        self.odom_x = list()
        self.odom_y = list()
        self.odom_yaw = list()
        self.clear = False
        self.path_received = False

    def get_path(self):
        return self.constructed_path

    def reset(self):
        self.constructed_path.poses = list()

    def odomCB(self,msg):
        new_pose = PoseStamped()
        new_pose.pose = msg.pose.pose
        self.constructed_path.poses.append(new_pose)

class ExecutionAnalyzer:
    def __init__(self):
        self.tss = ApproximateTimeSynchronizer([Subscriber("/odom",Odometry), Subscriber("/imu/data_raw_transformed", Imu)],5,0.1)
        self.tss.registerCallback(self.got_velocities)
        self.accumulate_error = [0,0,0]
        self.samples_number = 0
        self.accumulated_vel = 0.0

    def reset(self):
        print "RESET ANALYZER"
        self.samples_number = 0
        self.accumulate_error = [0.,0.,0.]
        self.accumulated_vel = 0.0

    def get_accumulated_error(self):
        return self.accumulate_error

    def get_mean_accumulated_error(self):
        if self.samples_number > 0:
            return [x / self.samples_number for x in self.accumulate_error]
        rospy.logwarn("SAMPLES NUMBER IS 0")
        return self.accumulate_error

    def got_velocities(self, odom, imu):
        if imu.linear_acceleration.x > 0.08 or np.fabs(imu.angular_velocity.z) > 0.008:
            self.accumulated_vel+= imu.linear_acceleration.x
            diff_x = self.accumulated_vel - odom.twist.twist.linear.x
            diff_z = imu.angular_velocity.z - odom.twist.twist.angular.z
            diff_yaw = yaw(imu.orientation) - yaw(odom.pose.pose.orientation)
            self.accumulate_error[0] += diff_x
            self.accumulate_error[1] += diff_z
            self.accumulate_error[2] += diff_yaw
            self.samples_number+=1
