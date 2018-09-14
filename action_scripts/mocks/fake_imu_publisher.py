import rospy
from sensor_msgs.msg import Imu

rospy.init_node("fake_imu", anonymous=False)
pub = rospy.Publisher("/imu/data_raw_transformed", Imu, queue_size =4)
msg = Imu()
rate = rospy.Rate(100)
 
while not rospy.is_shutdown():
    msg.header.stamp = rospy.Time.now()
    pub.publish(msg)
    rate.sleep()
