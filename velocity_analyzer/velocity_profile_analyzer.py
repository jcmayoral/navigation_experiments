import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
from message_filters import Subscriber, ApproximateTimeSynchronizer

last_odom_vel = Odometry()
last_cmd_vel = Twist()
max_error = Twist()
max_commanded_accel = Twist()
max_reported_accel = Twist()

diffSpeed = lambda cmd_vel, odom_vel: Twist(linear=Vector3(x=cmd_vel.linear.x - odom_vel.twist.twist.linear.x),
                                            angular=Vector3(z=cmd_vel.angular.z - odom_vel.twist.twist.angular.z))

getAccel = lambda cmd_vel, last_cmd_vel: Twist(linear=Vector3(x=cmd_vel.linear.x - last_cmd_vel.linear.x),
                                            angular=Vector3(z=cmd_vel.angular.z - last_cmd_vel.angular.z))

getOdomAccel = lambda odom_vel, last_odom_vel: Twist(linear=Vector3(x=odom_vel.twist.twist.linear.x - last_odom_vel.twist.twist.linear.x),
                                            angular=Vector3(z=odom_vel.twist.twist.angular.z - last_odom_vel.twist.twist.angular.z))


def velocities_cb(odom_msg, cmd_msg):
    global max_error
    diff_twist = diffSpeed(cmd_msg, odom_msg)
    max_error.linear.x = get_max(diff_twist.linear.x, max_error.linear.x)
    max_error.angular.z = get_max(diff_twist.angular.z, max_error.angular.z)


def get_max (val_a, val_b):
    if val_a > val_b:
        return val_a
    else:
        return val_b

def cmd_cb(cmd_msg):
    global last_odom_vel, max_error
    diff_twist = diffSpeed(cmd_msg, last_odom_vel)
    max_error.linear.x = get_max(diff_twist.linear.x, max_error.linear.x)
    max_error.angular.z = get_max(diff_twist.angular.z, max_error.angular.z)

    global max_commanded_accel, last_cmd_vel
    diff_twist = getAccel(cmd_msg, last_cmd_vel)
    max_commanded_accel.linear.x = get_max(diff_twist.linear.x, max_commanded_accel.linear.x)
    max_commanded_accel.angular.z = get_max(diff_twist.angular.z, max_commanded_accel.angular.z)
    last_cmd_vel = cmd_msg

def odom_cb(odom_msg):
    global last_odom_vel, max_reported_accel
    diff_twist = getOdomAccel(odom_msg, last_odom_vel)
    max_reported_accel.linear.x = get_max(diff_twist.linear.x, max_reported_accel.linear.x)
    max_reported_accel.angular.z = get_max(diff_twist.angular.z, max_reported_accel.angular.z)
    last_odom_vel = odom_msg


rospy.init_node("Velocity_Profile_Analyzer")

rospy.Subscriber("/odom", Odometry, odom_cb)
rospy.Subscriber("/cmd_vel", Twist, cmd_cb)
tss = ApproximateTimeSynchronizer([Subscriber("/odom",Odometry), Subscriber("/cmd_vel", Twist)],5,0.1, allow_headerless=True)
tss.registerCallback(velocities_cb)
rospy.spin()

print "MAX ERROR", max_error
print "MAX COMMANDED ACCEL", max_commanded_accel
print "MAX REPORTED ACCEL", max_reported_accel
