import tf
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import numpy as np

class StraightOdomMotion:
    def __init__(self, distance = 1.0, global_frame="odom", desired_speed=0.1, dist_threshold=0.1):
        rospy.init_node("straight_motion")
        self.global_frame = global_frame
        self.dist_threshold = dist_threshold
        self.goal_pose = np.array((0,0))
        self.distance = distance
        self.is_goal_reached = False
        self.desired_speed = 0.1
        self.sub = rospy.Subscriber("/odom", Odometry, self.odometry_cb)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.listener = tf.TransformListener()

    def odometry_cb(self, msg):
        self.listener.waitForTransform(self.global_frame, "base_link", rospy.Time(0), rospy.Duration(.5))
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x = msg.pose.pose.position.x
        pose.pose.position.y = msg.pose.pose.position.y
        current_pose, current_q = self.listener.transformPose("base_link", pose)
        current_pose = np.array((current_pose.position.x, current_pose.position.y))
        print np.linalg.norm(self.goal_pose-current_pose)
        if np.linalg.norm(self.goal_pose-current_pose) < self.dist_threshold:
            self.is_goal_reached = True

    def publish_speed(self):
        if self.is_goal_reached:
            return False
        speed = Twist()
        speed.linear.x = self.desired_speed
        self.pub.publish(speed)
        return True

    def calculate_end_pose(self):
        self.listener.waitForTransform(self.global_frame, "base_link", rospy.Time(0), rospy.Duration(.5))
        start_position, start_quaternion = self.listener.lookupTransform(self.global_frame, "base_link", rospy.Time(0))
        self.goal_pose = np.array((start_position[0] + 1.0, start_position[1]))

if __name__ == '__main__':
    straight_odom_motion = StraightOdomMotion()

    while not straight_odom_motion.is_goal_reached:
        straight_odom_motion.publish_speed()
