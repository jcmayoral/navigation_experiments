import rospy
from nav_msgs.msg import Path,Odometry
from std_msgs.msg import Empty, Boolx`
from geometry_msgs.msg import Vector3, PoseStamped
import numpy as np
from scipy.optimize import minimize, curve_fit, least_squares
from sbpl_primitives_analysis import SBPLPrimitiveAnalysis
import time
import tf2_ros
import tf2_geometry_msgs

#TODO Several poses (Integrate topological graph planner)

class ContractNetTimeEstimator(SBPLPrimitiveAnalysis):
    def __init__(self):
        #rospy.init_node("time_estimator")
        SBPLPrimitiveAnalysis.__init__(self)
        self.mean_primitive_error = 0
        self.samples = 0
        self.is_training = True
        self.primitives_number = 25
        self.feedback_pub = rospy.Publisher("time_estimator/fb", Vector3, queue_size=1)
        self.start_time = rospy.Time.now()
        self.estimated_time = 0
        self.lst_estimated_time = None
        self.lenght = 1
        self.K =0
        self.counter = 0
        self.A = list()
        self.y = list()
        self.A_primitives = list()
        self.y_diff = list()
        self.W = list()
        self.coefficients = np.zeros(6)
        self.min_coefficients = np.zeros(6)
        self.primitives_count = np.zeros(self.primitives_number)
        self.primitives_coefficients = np.zeros(self.primitives_number)
        self.timed_positions = list()
        self.is_robot_moving = False
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tfBuffer)

        rospy.Timer(rospy.Duration(0.5), self.timer_cb)
        #rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=1)

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
        mean = np.mean(self.y)
        x = [0,0,0,0,0,0]
        #self.coefficients = least_squares(f, x, ftol=np.mean(self.y_diff)).x
        self.coefficients = np.linalg.lstsq(self.A, self.y)[0]
        f = lambda x: np.fabs(np.sum(self.coefficients*x)) - mean
        #self.min_coefficients = minimize(f,x, tol=mean).x
        if self.counter == len(self.coefficients):
            self.min_coefficients += np.linalg.solve(self.A[-6:], self.y[-6:])/2.0
            self.counter = 0
        self.counter += 1
        print "new coefficients" , self.coefficients
        print "new coefficients" , self.min_coefficients



        #TODO
        prim_a = np.asarray(self.A_primitives)
        prim_mean = np.mean(self.y)
        x = np.zeros(self.primitives_number)
        print "Primitives coefficients " , np.linalg.lstsq(prim_a, self.y)[0]
        self.reset_primitives_count()
        self.primitives_count = np.zeros(self.primitives_number)

    def timer_cb(self, event):
        if self.is_robot_moving and len(self.timed_positions) > 0:
            if (time.time() - self.init_time) > self.timed_positions[0][0]:
                print "time elapsed",  time.time() - self.init_time
                expected_pose = self.timed_positions.pop(0)
                odom_pose = rospy.wait_for_message("odom", Odometry).pose
                odom_pose_stamped = PoseStamped()
                odom_pose_stamped.header.stamp = rospy.Time.now()
                odom_pose_stamped.pose = odom_pose.pose

                transform = self.tfBuffer.lookup_transform("map",
                                       "odom", #source frame
                                       rospy.Time(0), #get the tf at first available time
                                       rospy.Duration(1.0)) #wait for 1 second
                pose_transformed = tf2_geometry_msgs.do_transform_pose(odom_pose_stamped, transform)
                #print "Expected time ", expected_pose[0]
                #print "Expected pose ", pose_transformed
                #print "Odom pose", odom_pose
                diff_x = pose_transformed.pose.position.x- expected_pose[1].position.x
                diff_y = pose_transformed.pose.position.x- expected_pose[1].position.x
                if np.sqrt(np.power(diff_x,2) + np.power(diff_y,2)) > 1:
                    rospy.logerr("ROBOT getting delayed")

    def start_timer(self):
        rospy.loginfo("start timer")
        self.init_time = time.time()
        self.is_robot_moving = True

    def stop_timer(self):
        rospy.loginfo("stop timer")
        self.is_robot_moving = False
        self.init_time = time.time()

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
        self.K = [dx,dy,ddx,ddy,curvature, self.lenght]
        #print "Curvature " , self.K
        statistic_estimation = 0

        # For sbpl primitives
        for i in range(self.primitives_number):
            self.primitives_count[i] = self.get_primitive_count(i)
        print "Current primitives ", self.current_results

        if self.samples >0 :# not self.is_training:
            statistic_estimation = self.estimated_time + self.lenght* self.mean_primitive_error/self.samples
            rospy.logwarn("Statistical Estimation %f" , statistic_estimation)

        else:
            statistic_estimation = self.estimated_time

        primitive_estimation = self.estimated_time + np.sum(self.primitives_coefficients *self.primitives_count)
        rospy.logwarn("Estimation with primitives %f ", primitive_estimation)
        time_segments = round(primitive_estimation)
        time_lapse = np.arange(0,time_segments,1) #one check pose every second

        self.timed_positions = list()

        for t in time_lapse:
            tmp_time = primitive_estimation * t/time_segments
            tmp_pose = msg.poses[int(self.lenght * t/time_segments)].pose
            #print " At time %f robot should be at pose " % tmp_time
            #print tmp_pose
            self.timed_positions.append([tmp_time, tmp_pose])

        self.lst_estimated_time = self.estimated_time + np.sum(self.coefficients * np.array([dx, dy, ddx, ddy,curvature, self.lenght]))
        print "Complete Linearization Estimation " , self.lst_estimated_time
        print "Curvature Linearization Estimation " , self.estimated_time + np.sum(self.coefficients[4] * np.array([curvature]))
        return (statistic_estimation + self.lst_estimated_time)/2

    def is_motion_finished(self):
        self.is_robot_moving = False
        measured_time = (rospy.Time.now() - self.start_time).to_sec()

        if self.is_training:
            self.samples = self.samples+1
            self.mean_primitive_error += (measured_time - self.estimated_time)/self.lenght

        rospy.logerr("MEASURED TIME %f",  measured_time)
        #print "ERROR IN SECONDS", self.estimated_time - measured_time
        new_measurement = self.K
        #new_measurement.append(self.estimated_time)
        self.A.append(new_measurement)
        self.A_primitives.append(self.primitives_count)
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
