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

#TODO Several poses (Integrate topological graph planner)

class ContractNetTimeEstimator(SBPLPrimitiveAnalysis):
    def __init__(self):
        #rospy.init_node("time_estimator")
        SBPLPrimitiveAnalysis.__init__(self)
        self.is_robot_moving = False
        self.distance_tolerance = 1000 # disable
        self.prediction_time = 0.25
        self.mean_primitive_error = 0
        self.samples = 0
        self.primitives_number = 25
        self.start_time = rospy.Time.now()
        self.estimated_time = 0
        self.lst_estimated_time = None
        self.lenght = 1
        self.features =0
        self.counter = 0
        self.A = list()
        self.A_primitives = list()
        self.y = list()
        self.W = list()
        self.coefficients = np.zeros(6)
        self.min_coefficients = np.zeros(6)
        self.primitives_count = np.zeros(self.primitives_number)
        self.primitives_coefficients = np.zeros(self.primitives_number)
        self.timed_positions = list()

        self.tfBuffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tfBuffer)
        self.feedback_pub = rospy.Publisher("/trajectory_estimator", PoseStamped, queue_size=1)
        rospy.Timer(rospy.Duration(self.prediction_time*4), self.timer_cb)

    def train_data(self):
        self.coefficients = np.linalg.lstsq(self.A, self.y)[0]

        self.primitives_coefficients = np.linalg.lstsq(self.A_primitives, self.y)[0]

        #reset_counter
        self.reset_primitives_count()

    def timer_cb(self, event):
        if self.is_robot_moving and len(self.timed_positions) > 0:
            selected_index = 0
            for i in range(len(self.timed_positions)):
                if time.time() - self.init_time > self.timed_positions[i][0]:
                    selected_index = i
                    break

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

        if self.samples >0 :
            statistic_estimation = self.estimated_time + self.lenght* self.mean_primitive_error/self.samples
            rospy.logwarn("Statistical Estimation %f" , statistic_estimation)

        else:
            statistic_estimation = self.estimated_time

        #This approache takes longer to converge
        self.primitive_estimation = np.sum(self.primitives_coefficients *self.primitives_count)
        rospy.logwarn("Estimation with primitives %f ", self.primitive_estimation)

        #time variables used for predicition
        time_segments = self.primitive_estimation
        time_lapse = np.arange(0,time_segments,self.prediction_time) #one check pose every second

        self.timed_positions = list()
        current_primitives = self.get_primitive_list()
        progressive_costs = list()
        total_cost = 0

        #accumulate cost -> we could know which primitive takes more time to execute
        #NOTE with additional knowledge we could know the execution time per primitive
        for prim in current_primitives:
            try:
                #getting cost per primitive
                prim_cost = int(prim) * self.primitives_coefficients[int(prim)]
                total_cost += prim_cost
            except ValueError:
                pass
            progressive_costs.append(total_cost) #ignoring front_search, back_search values

        expected_cost = 0
        min_range = 0
        for t  in time_lapse:
            if time_segments > 0:
                tmp_time = t #expected time
                tmp_pose = msg.poses[0].pose
                flag = False
                for c in range(min_range,len(progressive_costs)):
                    #mapping path poses with percentage of the total_cost
                    if progressive_costs[c] > expected_cost and not flag:
                        tmp_pose = msg.poses[int(self.lenght * c/len(progressive_costs))].pose
                        expected_cost = progressive_costs[c]
                        min_range = c
                        flag = True
                if flag:
                    self.timed_positions.append([tmp_time, tmp_pose])

        self.lst_estimated_time = np.sum(self.coefficients * np.array([dx, dy, ddx, ddy,curvature, self.lenght]))
        rospy.logwarn("Complete Linearization Estimation %f " % self.lst_estimated_time)
        return (statistic_estimation + self.lst_estimated_time)/2

    def is_motion_finished(self):
        self.is_robot_moving = False

        #Used for statistical estimation
        self.samples = self.samples+1
        measured_time = (rospy.Time.now() - self.start_time).to_sec()
        self.mean_primitive_error += (measured_time - self.estimated_time)/self.lenght

        rospy.logerr("MEASURED TIME %f",  measured_time)
        new_measurement = self.features

        #used for Linearization
        self.A.append(new_measurement)

        #used for primitive base linearization
        self.A_primitives.append(self.primitives_count)
        self.y.append(measured_time)

        #TODO Weighting
        self.W.append(1)
        self.train_data()
