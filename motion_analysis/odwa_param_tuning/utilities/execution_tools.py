import tf
import rospy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist, Vector3
from message_filters import TimeSynchronizer, Subscriber, ApproximateTimeSynchronizer
from mag_common_py_libs.geometry import yaw

class PathConstructor:
    def __init__(self, path_frame = "odom"):
        rospy.Subscriber("/odom",Odometry,self.odomCB)
        self.constructed_path = Path()
        self.constructed_path.header.frame_id = path_frame

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
        self.tss = ApproximateTimeSynchronizer([Subscriber("/odom",Odometry), Subscriber("/cmd_vel", Twist)],5,0.1, allow_headerless=True)
        self.tss.registerCallback(self.got_velocities)
        self.samples_number = 0
        self.accumulated_vel = Twist()
        #Defining Operators between Twist messgaes
        self.accumulated_error = [0,0]
        self.accumulateSpeed = lambda t1, t2: Twist(linear=Vector3(x=t1.linear.x + t2.linear.x), angular=Vector3(z=t1.angular.z + t2.angular.z))
        self.diffSpeed = lambda t1, t2: Twist(linear=Vector3(x=t1.linear.x - t2.linear.x), angular=Vector3(z=t1.angular.z - t2.angular.z))

    def reset(self):
        self.samples_number = 0
        self.accumulated_error = [0.,0.]
        self.accumulated_vel = Twist()

    def get_accumulated_error(self):
        self.accumulated_error[0] = self.accumulated_vel.linear.x
        self.accumulated_error[1] = self.accumulated_vel.angular.z
        return self.accumulated_error

    def get_accumulated_velocities(self):
        return [self.accumulated_vel.linear.x, self.accumulated_vel.angular.z]

    def get_mean_accumulated_error(self):
        self.accumulated_error[0] = self.accumulated_error.linear.x
        self.accumulated_error[1] = self.accumulated_error.angular.z

        if self.samples_number > 0:
            return [x / self.samples_number for x in self.accumulated_error]
        rospy.logwarn("SAMPLES NUMBER IS 0")
        return self.accumulated_error

    def got_velocities(self, odom, current_vel):
        self.accumulated_vel= self.accumulateSpeed(self.accumulated_vel, current_vel)
        diff_x = current_vel.linear.x - odom.twist.twist.linear.x
        diff_z = current_vel.angular.z - odom.twist.twist.angular.z
        self.accumulated_error[0] += diff_x
        self.accumulated_error[1] += diff_z
        self.samples_number+=1
