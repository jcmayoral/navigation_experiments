import rospy
from nav_msgs.msg import Path,Odometry
from std_msgs.msg import Empty, Bool
from geometry_msgs.msg import Vector3, PoseStamped
import numpy as np
from scipy.optimize import minimize, curve_fit, least_squares
from sbpl_primitives_analysis import SBPLPrimitiveAnalysis
import time
import tf2_ros
import tf2_geometry_msgs
import copy
from sklearn import linear_model


#TODO Several poses (Integrate topological graph planner)

class ContractNetTimeEstimator(SBPLPrimitiveAnalysis):
    def __init__(self):
        #rospy.init_node("time_estimator")
        SBPLPrimitiveAnalysis.__init__(self)
        self.is_robot_moving = False
        self.distance_tolerance = 1000 # disable
        self.prediction_time = 1.0
        self.mean_primitive_error = 0
        self.samples = 0
        self.primitives_number = 25
        self.start_time = rospy.Time.now()
        self.estimated_time = 0
        self.lst_estimated_time = None
        self.lenght = 1
        self.features =0
        self.counter = 0
        self.mean_expected_time = 0.0
        self.A = list()
        self.A_primitives = list()
        self.y = list()
        self.W = list()
        self.coefficients = np.zeros(6)
        self.min_coefficients = np.zeros(6)
        self.primitives_count = np.zeros(self.primitives_number)
        self.primitives_coefficients = np.zeros(self.primitives_number)
        self.timed_positions = list()
        self.ransac = linear_model.RANSACRegressor(min_samples = 1, residual_threshold = 0.5)
        self.primitive_ransac = linear_model.RANSACRegressor(min_samples = 1, residual_threshold = 1)
        self.ransac_fit = False
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tfBuffer)
        self.feedback_pub = rospy.Publisher("/trajectory_estimator", PoseStamped, queue_size=1)
        rospy.Timer(rospy.Duration(self.prediction_time/2), self.timer_cb)

    def train_data(self):
        n_samples = int(np.floor(len(self.y)*0.6))
        n_samples = 1 if n_samples < 1 else n_samples

        self.ransac = linear_model.RANSACRegressor(min_samples = n_samples, residual_threshold = 0.5)
        self.primitive_ransac = linear_model.RANSACRegressor(min_samples = n_samples, residual_threshold = 0.5)
        self.ransac.fit(np.array(self.A), np.array(self.y).reshape(len(self.y), 1))
        self.primitive_ransac.fit(np.array(self.A_primitives), np.array(self.y).reshape(len(self.y), 1))
        self.ransac_fit = True

        self.coefficients = np.linalg.lstsq(self.A, self.y)[0]
        self.primitives_coefficients = np.linalg.lstsq(self.A_primitives, self.y)[0]

        #reset_counter
        self.reset_primitives_count()

    def timer_cb(self, event):
        if self.is_robot_moving and len(self.timed_positions) > 0:
            selected_index = 0
            current_time = time.time() - self.init_time
            for i in range(len(self.timed_positions)):
                if current_time > self.timed_positions[i][0]:
                    selected_index = i

            del self.timed_positions[0:selected_index]

            t, expected_pose = self.timed_positions.pop(0)

            #Convert pose from odom to map frame
            odom_pose = rospy.wait_for_message("odom", Odometry).pose
            odom_pose_stamped = PoseStamped()
            odom_pose_stamped.header.stamp = rospy.Time.now()
            odom_pose_stamped.pose = odom_pose.pose

            transform = self.tfBuffer.lookup_transform("map",
                                   "odom", #source frame
                                   rospy.Time(0), #get the tf at first available time
                                   rospy.Duration(1.0)) #wait for 1 second
            pose_transformed = tf2_geometry_msgs.do_transform_pose(odom_pose_stamped, transform)

            #publishing expected position on map frame
            fb_msgs = PoseStamped()
            fb_msgs.header.frame_id = "map"
            fb_msgs.header.stamp = rospy.Time.now()
            fb_msgs.pose = expected_pose
            self.feedback_pub.publish(fb_msgs)

            #What is the difference between expected and real
            diff_x = pose_transformed.pose.position.x- expected_pose.position.x
            diff_y = pose_transformed.pose.position.y- expected_pose.position.y
            bias = np.sqrt(np.power(diff_x,2) + np.power(diff_y,2))
            if bias > self.distance_tolerance:
                rospy.logerr("ROBOT out of plan by %f" % bias)

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
        self.start_time = rospy.Time.now()

        p0 = msg.poses[0]
        self.features = 0

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
        self.features = [dx,dy,ddx,ddy,curvature, self.lenght]

        statistic_estimation = 0

        # For sbpl primitives , get how many primitive of i type are on the path
        self.primitives_count = np.zeros(self.primitives_number)
        for i in range(self.primitives_number):
            self.primitives_count[i] = self.get_primitive_count(i)

        #HACK  TODO RIP primitives store on the last value
        self.primitives_count[self.primitives_number-1] = self.get_primitive_count('rip')


        if self.samples >0 :
            statistic_estimation = self.estimated_time + self.lenght* self.mean_primitive_error/self.samples
            rospy.logwarn("Statistical Estimation %f" , statistic_estimation)

        else:
            statistic_estimation = self.estimated_time

        #This approache takes longer to converge
        self.primitive_estimation = np.sum(self.primitives_coefficients *self.primitives_count)
        rospy.logwarn("Estimation with primitives %f ", self.primitive_estimation)

        self.lst_estimated_time = np.sum(self.coefficients * np.array([dx, dy, ddx, ddy,curvature, self.lenght]))
        rospy.logwarn("Complete Linearization Estimation %f " % self.lst_estimated_time)

        ransac_primitive_estimatiom = 0
        ransac_lst_estimation = 0

        if self.ransac_fit:
            ransac_primitive_estimatiom = self.primitive_ransac.predict(self.primitives_count.reshape(1,-1))
            ransac_lst_estimation = self.ransac.predict(np.array([dx, dy, ddx, ddy,curvature, self.lenght]).reshape(1, -1))
            rospy.logwarn("Primitives with ransac %f" % ransac_primitive_estimatiom)
            rospy.logwarn("Linearization with ransac %f" % ransac_lst_estimation)

        results_array = np.array([self.lst_estimated_time , self.primitive_estimation, statistic_estimation, ransac_primitive_estimatiom , ransac_lst_estimation])
        std_dev = np.std(results_array)
        rospy.loginfo("Standard Deviation %f " % std_dev)
        self.mean_expected_time = np.sum(results_array)/5
        rospy.logerr("Average expected %f seconds" % self.mean_expected_time)

        #time variables used for predicition
        time_lapse = np.arange(0,self.primitive_estimation+self.prediction_time,self.prediction_time) #one check pose every second
        self.timed_positions = list()
        current_primitives = self.get_primitive_list()
        progressive_costs = list()
        total_cost = 0

        #accumulate cost -> we could know which primitive takes more time to execute
        #NOTE with additional knowledge we could know the execution time per primitive
        for prim in current_primitives:
            try:
                #getting cost per primitive
                if "rip" in prim:
                    print "rip find"
                    prim_cost = int(prim) * self.primitives_coefficients[self.primitives_number-1]
                else:
                    prim_cost = int(prim) * self.primitives_coefficients[int(prim)]
                total_cost += prim_cost
            except ValueError:
                pass
            progressive_costs.append(total_cost) #ignoring front_search, back_search values

        counter = 0

        #mapping time with cost
        #main assumption time and costs are linear not exponential
        #TODO mean value of coefficients -> problem each model has different number of coefficients
        for t  in time_lapse:

            """
            ### Protection for cost based approach
            if counter == len(progressive_costs) -1:
                break
            """
            index = int(self.lenght * t /self.mean_expected_time)

            if self.samples > 1:
                index = int (index * (1-self.mean_primitive_error))

            if index >= self.lenght - 1:
                break

            #This approach is simpler mapping the time lapse and lenght of Path
            #Drift over time, a correction is required
            tmp_pose = msg.poses[index].pose
            self.timed_positions.append([t, tmp_pose])

            """
            #This approach should select the expected pose according to cost but cost seems not to be proportional
            #Linearization not warranty, this

            tmp_pose = msg.poses[int(self.lenght * counter)/len(progressive_costs)].pose
            self.timed_positions.append([t, tmp_pose])

            while progressive_costs[counter]/total_cost < t/time_lapse[-1]:
                counter = counter + 1
                if counter == len(progressive_costs) -1:
                    break
            """

        return self.mean_expected_time

    def is_motion_finished(self):
        self.is_robot_moving = False

        #Used for statistical estimation
        self.samples = self.samples+1
        measured_time = (rospy.Time.now() - self.start_time).to_sec()
        self.mean_primitive_error += (measured_time - self.estimated_time)/self.lenght

        rospy.logerr("Measured Time %f",  measured_time)
        rospy.logwarn("Accuracy %f", np.fabs(measured_time - self.mean_expected_time)/measured_time)
        new_measurement = self.features

        #used for Linearization

        self.A.append(new_measurement)

        #used for primitive base linearization
        self.A_primitives.append(self.primitives_count)
        self.y.append(measured_time)

        #TODO Weighting
        mean_error = np.fabs(measured_time - self.estimated_time)
        mean_error += np.fabs(measured_time - self.lst_estimated_time)
        mean_error += np.fabs(measured_time - self.primitive_estimation)
        mean_error = mean_error/(3 * measured_time)

        self.W.append(mean_error)
        self.train_data()
