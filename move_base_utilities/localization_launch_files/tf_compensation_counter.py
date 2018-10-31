#!/usr/bin/python
import tf
from geometry_msgs.msg import Quaternion
import rospy
from tf.transformations import euler_from_quaternion
from numpy import fabs, sqrt, power
from std_msgs.msg import Float32, Bool
from nav_msgs.msg import Odometry

stop_flag = False

def stop_cb(msg):
    print "Ehgpiashgpshapigsahpahgpshgdpi"
    stop_flag = msg.data

rospy.init_node("cartographer_tf_interventions")

map_frame = "map_carto"
odom_frame = "odom"

fb_publish = rospy.Publisher("tf_error", Float32, queue_size = 5)
fb_angular_publish = rospy.Publisher("tf_angular_error", Float32, queue_size = 5)
#rospy.Subscriber("stop_grading", Bool, stop_cb)

first_odom = rospy.wait_for_message('/odom', Odometry)

listener = tf.TransformListener()
listener.waitForTransform(map_frame, odom_frame, rospy.Time(0), rospy.Duration(10.0))
old_pose, old_orientation = listener.lookupTransform(map_frame, odom_frame, rospy.Time(0))
old_time = rospy.Time.now()
tf_corrections = 0
accumulated_error = 0.0
angular_accumulated_error = 0.0
detected_delays = 0
tolerance = -1.0
ang_tolerance = -1.0
max_delay = 0.05

while not rospy.is_shutdown():
    print stop_flag
    try:
        current_odom = rospy.wait_for_message('/odom', Odometry)
    except Exception as e:
        print "Shutdown"
    listener.waitForTransform(map_frame, odom_frame, rospy.Time(0), rospy.Duration(1.0))
    p, o = listener.lookupTransform(map_frame, odom_frame, rospy.Time(0))

    pose_diff = [True if fabs(x-y)> tolerance else False for (x,y) in zip(p,old_pose)]
    #pose_diff = [fabs(x - y) for (x, y) in zip(p, old_pose)]
    orientation_diff = fabs(euler_from_quaternion(o)[2] - euler_from_quaternion(old_orientation)[2])
    new_time = rospy.Time.now()
    fb_msg = Float32(data=0)
    fb_angular_msg = Float32(data=0)

    if any(pose_diff) > tolerance or orientation_diff > ang_tolerance:
        rospy.logwarn("TF Correction Found %s %s ", (pose_diff), orientation_diff)
        accumulated_error += sqrt(power(p[0]-old_pose[0],2) + power(p[1]-old_pose[1],2))
        angular_accumulated_error += orientation_diff
        tf_corrections+=1
        fb_msg.data = accumulated_error
        fb_angular_msg.data = angular_accumulated_error

    if (new_time - old_time).to_sec()> max_delay:
        rospy.logwarn("Delay")
        detected_delays+=1
        #fb_msg.data = 0.7

    fb_publish.publish(fb_msg)
    fb_angular_publish.publish(fb_angular_msg)
    old_pose = p
    old_orientation = o
    old_time = new_time

rospy.loginfo("Odom diff x %f", current_odom.pose.pose.position.x - first_odom.pose.pose.position.x)
rospy.loginfo("Odom diff y %f", current_odom.pose.pose.position.y - first_odom.pose.pose.position.y)
o1 = [current_odom.pose.pose.orientation.x, current_odom.pose.pose.orientation.y, current_odom.pose.pose.orientation.z, current_odom.pose.pose.orientation.w]
o2 = [first_odom.pose.pose.orientation.x, first_odom.pose.pose.orientation.y, first_odom.pose.pose.orientation.z, first_odom.pose.pose.orientation.w]
rospy.loginfo("Odom diff orientation %f", euler_from_quaternion(o1)[2] - euler_from_quaternion(o2)[2])
rospy.loginfo("Accumulated error %f", accumulated_error)
rospy.loginfo("Accumulated angular error %f", angular_accumulated_error)
rospy.loginfo("tf corrections found %i", tf_corrections)
rospy.loginfo("delays found %i", detected_delays)
