import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Empty, Bool
from geometry_msgs.msg import Vector3
import numpy as np
from scipy.optimize import minimize, curve_fit, least_squares

#TODO Several poses (Integrate topological graph planner)

class ContractNetTimeEstimator:
    def __init__(self):
        #rospy.init_node("time_estimator")
        self.mean_primitive_error = 0
        self.samples = 0
        self.is_training = True
        #rospy.Subscriber("/time_estimator/training", Bool, self.training_cb, queue_size=1)
        self.feedback_pub = rospy.Publisher("time_estimator/fb", Vector3, queue_size=1)
        self.start_time = rospy.Time.now()
        self.estimated_time = 0
        self.lst_estimated_time = None
        self.lenght = 1
        self.K =0
        self.A = list()
        self.y = list()
        self.y_diff = list()
        self.W = list()
        self.coefficients = np.zeros(5)
        self.min_coefficients = np.zeros(5)
        #rospy.spin()

    def train_data(self):
        #print "OFF SET per primitive is ", self.mean_primitive_error/self.samples

        #offset = np.ones(len(self.y))
        #self.C = np.vstack([self.A, offset]).T
        """
        if len(self.W) > 1:
            W = np.sqrt(np.diag(self.W))
            Aw = np.dot(W,self.A)
            Bw = np.dot(W,self.y)
        """

        #print "new coefficients "  , self.coefficients

        #TODO TOTEST XXX
        a = np.asarray(self.A)
        mean = np.mean(self.y_diff)
        f = lambda x: np.fabs(np.sum(self.coefficients*x) - mean)
        x = [0,0,0,0,0]
        self.min_coefficients = minimize(f,x).x
        #self.coefficients = least_squares(f, x, ftol=np.mean(self.y_diff)).x
        self.coefficients = np.linalg.lstsq(self.A, self.y)[0]

        #print self.coefficients

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
            dx.append(np.fabs(p.pose.position.x - p0.pose.position.x))
            dy.append(np.fabs(p.pose.position.y - p0.pose.position.y))
            p0 = p

        dx0 = dx[0]
        dy0 = dy[0]

        for x,y in zip(dx[1:],dy[1:]):
            ddx.append(np.fabs(x - dx0))
            ddy.append(np.fabs(y - dy0))
            dx0 = x
            dy0 = y

        dx = np.mean(dx,axis=0)
        dy = np.mean(dy,axis=0)
        ddx = np.mean(ddx, axis=0)
        ddy = np.mean(ddy,axis=0)

        curvature = (ddy * dx - ddx * dy) / (np.power(dx, 2) + np.power(dy, 2))
        self.K = [dx,dy,ddx,ddy,curvature]
        #print "Curvature " , self.K
        statistic_estimation = 0

        if self.samples >0 :# not self.is_training:
            statistic_estimation = self.estimated_time + self.lenght* self.mean_primitive_error/self.samples
            print "Statistical Estimation " , statistic_estimation

        else:
            statistic_estimation = self.estimated_time

        self.lst_estimated_time = self.estimated_time + np.sum(self.coefficients * np.array([dx, dy, ddx, ddy,curvature]))
        print "Complete Linearization Estimation " , self.lst_estimated_time
        print "Curvature Linearization Estimation " , self.estimated_time + np.sum(self.coefficients[4] * np.array([curvature]))
        #print "Minimizing ", self.estimated_time +  np.sum(self.min_coefficients * np.array([dx, dy, ddx, ddy, curvature]))
        #print "Minimizing ", -np.mean(self.y_diff) +  np.sum(self.min_coefficients * np.array([dx, dy, ddx, ddy, curvature]))

        return (statistic_estimation + self.lst_estimated_time)/2

    def is_motion_finished(self):
        measured_time = (rospy.Time.now() - self.start_time).to_sec()

        if self.is_training:
            self.samples = self.samples+1
            self.mean_primitive_error += (measured_time - self.estimated_time)/self.lenght

        print "MEASURED TIME", measured_time
        #print "ERROR IN SECONDS", self.estimated_time - measured_time
        new_measurement = self.K
        #new_measurement.append(self.estimated_time)
        self.A.append(new_measurement)
        self.y.append(measured_time)
        self.y_diff.append(measured_time - self.estimated_time)

        #TODO Weighting lstsq not working properly
        self.W.append(1)

        fb_msgs = Vector3()
        fb_msgs.x = abs(self.estimated_time - measured_time)/measured_time
        fb_msgs.y = self.lenght
        fb_msgs.z = self.K
        self.feedback_pub.publish(fb_msgs)
        self.train_data()
        #print "ERROR per primitive ", (self.estimated_time - measured_time)/self.lenght
