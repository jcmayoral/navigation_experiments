import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Empty, Bool
from geometry_msgs.msg import Vector3
import numpy as np

class ContractNetTimeEstimator:
    def __init__(self):
        #rospy.init_node("time_estimator")
        self.mean_primitive_error = 0
        self.samples = 0
        self.is_training = True
        rospy.Subscriber("/time_estimator/training", Bool, self.training_cb, queue_size=1)
        self.feedback_pub = rospy.Publisher("time_estimator/fb", Vector3, queue_size=1)
        self.start_time = rospy.Time.now()
        self.estimated_time = 0
        self.lenght = 1
        self.K =0
        self.A = list()
        self.y = list()
        self.coefficients = None
        #rospy.spin()

    def training_cb(self,path):
        self.is_training = msg.data
        if not self.is_training:
            print "OFF SET per primitive is ", self.mean_primitive_error/self.samples

        #offset = np.ones(len(self.y))
        #self.C = np.vstack([self.A, offset]).T
        self.coefficients = np.linalg.lstsq(self.A, self.y)[0]
        print self.coefficients

    def calculate_time(self,msg):
        self.lenght = len(msg.poses)
        self.estimated_time = 0.1*len(msg.poses)
        print "ORIGINAL ESTIMATION" , self.estimated_time
        self.start_time = rospy.Time.now()

        p0 = msg.poses[0]
        self.K = 0

        dx = list()
        dy = list()
        ddx = list()
        ddy = list()

        for p in msg.poses[1:]:
            dx.append(p.pose.position.x - p0.pose.position.x)
            dy.append(p.pose.position.y - p0.pose.position.y)
            p0 = p

        dx0 = dx[0]
        dy0 = dy[0]

        for x,y in zip(dx[1:],dy[1:]):
            ddx.append(x - dx0)
            ddy.append(y - dy0)
            dx0 = x
            dy0 = y

        dx = np.sum(dx,axis=0)
        dy = np.sum(dy,axis=0)
        ddx = np.sum(ddx, axis=0)
        ddy = np.sum(ddy,axis=0)

        self.K = (ddy * dx - ddx * dy) / (np.power(dx, 2) + np.power(dy, 2))
        #print "Curvature " , self.K
        statistic_estimation = 0

        if self.samples >0:# not self.is_training:
            statistic_estimation = self.estimated_time + self.lenght* self.mean_primitive_error/self.samples
            print "Statistical Estimation " , statistic_estimation

            if self.coefficients is not None:
                estimated_time = np.sum(self.coefficients * np.array([self.K , self.estimated_time]))
                print "Linearization Estimation " , estimated_time
        else:
            statistic_estimation = self.estimated_time

        return statistic_estimation

    def is_motion_finished(self,msg):
        measured_time = (rospy.Time.now() - self.start_time).to_sec()

        if self.is_training:
            self.samples = self.samples+1
            self.mean_primitive_error += (measured_time - self.estimated_time)/self.lenght

        print "MEASURED TIME", measured_time
        #print "ERROR IN SECONDS", self.estimated_time - measured_time
        self.A.append([self.K, self.estimated_time])
        self.y.append(measured_time)


        fb_msgs = Vector3()
        fb_msgs.x = abs(self.estimated_time - measured_time)/measured_time
        fb_msgs.y = self.lenght
        fb_msgs.z = self.K
        self.feedback_pub.publish(fb_msgs)
        #print "ERROR per primitive ", (self.estimated_time - measured_time)/self.lenght
