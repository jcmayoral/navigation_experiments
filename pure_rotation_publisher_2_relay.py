import tf
import rospy
from geometry_msgs.msg import PoseStamped
import sys

rospy.init_node("relay_publiser")

pub = rospy.Publisher("move_base_simple/goal", PoseStamped)
listener = tf.TransformListener()
listener.waitForTransform("map", "base_link", rospy.Time(0), rospy.Duration(.5))
position, o_quaternion = listener.lookupTransform("map", "base_link", rospy.Time(0))
print position, o_quaternion

goal = PoseStamped()
goal.header.stamp = rospy.Time.now()
goal.header.frame_id = "map"
goal.pose.position.x = position[0]
goal.pose.position.y = position[1]
roll, pitch, yaw = tf.transformations.euler_from_quaternion(o_quaternion)
quaternion = tf.transformations.quaternion_from_euler(0,0, yaw + float(sys.argv[1]))
goal.pose.orientation.x = quaternion[0]
goal.pose.orientation.y = quaternion[1]
goal.pose.orientation.z = quaternion[2]
goal.pose.orientation.w = quaternion[3]
 
pub.publish(goal)

print goal



