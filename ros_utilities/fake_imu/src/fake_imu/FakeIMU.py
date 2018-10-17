import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3, PoseWithCovarianceStamped, Quaternion
from tf import TransformListener

class FakeIMU:

    def __init__(self):
        rospy.init_node('fake_imu', anonymous=True)
        rospy.Subscriber("/fake_odom",Odometry, self.OdomCB)
        #rospy.Subscriber("/cmd_vel",Twist, self.twistCB)
        self.imu_Publisher = rospy.Publisher("/imu/data_raw", Imu, queue_size = 1)
        #self.odom_Publisher = rospy.Publisher("/fake_odom", Odometry, queue_size = 1)
        self.freq = 200
        self.tf = TransformListener()
        self.orientation = Quaternion()
        self.twist = Twist()
        self.is_shutdown = False
        self.last_values = [0,0,0]
        print "Fake_Imu Node Initialized"

    def twistCB(self,msg):
        self.twist = msg

    def OdomCB(self,msg):
        #msg.child_frame_id = "base_link"
        #msg.header.frame_id = "base_link"
        #self.odom_Publisher.publish(msg)
        self.orientation = msg.pose.pose.orientation
        self.twist = msg.twist.twist

    def publishImuMsg(self):
        #This code do not consider covariance
        tmp = Imu()
        tmp.header.frame_id = "base_link"
        #tmp.orientation = self.orientation

        #t = self.tf.getLatestCommonTime("/base_link", "/odom")
        self.tf.waitForTransform("map", "base_link", rospy.Time(0), rospy.Duration(1))
        self.tf.lookupTransform("/base_link", "/odom", rospy.Time(0))
        position, quaternion = self.tf.lookupTransform("/base_link", "/odom", rospy.Time(0))
        print type(quaternion)
        q = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])
        tmp.orientation = q

        tmp.linear_acceleration.x = self.twist.linear.x - self.last_values[0]
        tmp.linear_acceleration.y = self.twist.linear.y - self.last_values[1]
        tmp.angular_velocity.z = self.twist.angular.z - self.last_values[2]
        self.imu_Publisher.publish(tmp)

        self.last_values = [tmp.linear_acceleration.x , tmp.linear_acceleration.y, tmp.angular_velocity.z]

    def shutdown(self):
        self.is_shutdown = True
        print("Node Down")

    def run(self, frequency = 0):

        self.freq = frequency

        rospy.on_shutdown(self.shutdown)
        r = rospy.Rate(self.freq)

        while not self.is_shutdown:
            #print "Running"
            self.publishImuMsg()
            r.sleep()
        print ("Node Shutting down")
