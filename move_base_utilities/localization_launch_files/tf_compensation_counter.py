#!/usr/bin/python
import tf
from geometry_msgs.msg import Quaternion
import rospy
from tf.transformations import euler_from_quaternion
from numpy import fabs, sqrt, power
from std_msgs.msg import Float32

rospy.init_node("cartographer_tf_interventions")

map_frame = "map"
odom_frame = "odom"

fb_publish = rospy.Publisher("tf_error", Float32, queue_size = 5)
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
    listener.waitForTransform(map_frame, odom_frame, rospy.Time(0), rospy.Duration(1.0))
    p, o = listener.lookupTransform(map_frame, odom_frame, rospy.Time(0))

    pose_diff = [True if fabs(x-y)> tolerance else False for (x,y) in zip(p,old_pose)]
    #pose_diff = [fabs(x - y) for (x, y) in zip(p, old_pose)]
    orientation_diff = fabs(euler_from_quaternion(o)[2] - euler_from_quaternion(old_orientation)[2])
    new_time = rospy.Time.now()
    fb_msg = Float32(data=0)

    if any(pose_diff) > tolerance or orientation_diff > ang_tolerance:
        rospy.logwarn("TF Correction Found %s %s ", (pose_diff), orientation_diff)
        accumulated_error += sqrt(power(p[0]-old_pose[0],2) + power(p[1]-old_pose[1],2))
        angular_accumulated_error += orientation_diff
        tf_corrections+=1
        fb_msg.data = 0.5

    if (new_time - old_time).to_sec()> max_delay:
        rospy.logwarn("Delay")
        detected_delays+=1
        fb_msg.data = 0.7

    fb_publish.publish(fb_msg)
    old_pose = p
    old_orientation = o
    old_time = new_time

rospy.loginfo("Accumulated error %f", accumulated_error)
rospy.loginfo("Accumulated angular error %f", angular_accumulated_error)
rospy.loginfo("tf corrections found %i", tf_corrections)
rospy.loginfo("delays found %i", detected_delays)
