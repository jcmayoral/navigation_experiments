import rospy
import actionlib
from mbf_msgs.msg import MoveBaseResult, ExePathGoal, ExePathAction, GetPathAction, GetPathGoal, GetPathActionResult
from nav_msgs.msg import Path
from std_msgs.msg import Float64, String, Bool
from geometry_msgs.msg import PoseStamped
from time_estimator import ContractNetTimeEstimator

class RobotTimeEstimator:
    def __init__(self):
        rospy.init_node("time_estimator_standalone")
        self.name_id = "robot_0"
        self.is_busy = False
        self.response = "None"
        self.time_estimator = ContractNetTimeEstimator()

        self.get_path_ac = actionlib.SimpleActionClient("/navigation/move_base_flex/get_path", GetPathAction)
        self.exe_path_ac = actionlib.SimpleActionClient("/navigation/move_base_flex/exe_path", ExePathAction)

        self.rviz_fb = rospy.Subscriber('move_base_simple_time/goal', PoseStamped, self.simple_goal_cb)

        self.get_path_ac.wait_for_server(rospy.Duration(5))
        self.exe_path_ac.wait_for_server(rospy.Duration(5))
        rospy.loginfo("allcorrect")
        self.goal_pose = PoseStamped()
        self.path = Path()
        rospy.spin()

    def simple_goal_cb(self, msg):
        rospy.loginfo("CFP received from server")
        self.goal_pose = msg
        self.get_path()
        self.execute_path()
        self.time_estimator.is_motion_finished()


    def get_path(self):
        rospy.loginfo("Sending To get_path server")
        goal = GetPathGoal(use_start_pose=False,
                           target_pose=self.goal_pose)
        self.get_path_ac.send_goal(goal,done_cb = self.plan_done_cb)
        rospy.loginfo("Calling move_base_flex/get_path")
        self.get_path_ac.wait_for_result(timeout=rospy.Duration(10))
        rospy.loginfo("get_path is done")
        self.time_estimator.calculate_time(self.path)
        #TODO msg
        rospy.loginfo("Waiting for the response")

    def plan_done_cb(self,status, result):
        if result.outcome == MoveBaseResult.SUCCESS:
            self.path = result.path
            rospy.loginfo("Get path action succeeded")
        else:
            rospy.logerr("Get path action failed; outcome [%d], %s",
                          result.outcome, result.message)

    def execute_path(self):
        goal = ExePathGoal(path=self.path)
        self.exe_path_ac.send_goal(goal, done_cb=self.execution_done_cb)
        self.exe_path_ac.wait_for_result(timeout=rospy.Duration(120))
        return self.result

    def execution_done_cb(self,status, result):
        if result.outcome == MoveBaseResult.SUCCESS:
            rospy.loginfo("Exec path action succeeded")
            self.result = True
        else:
            rospy.logerr("Exec path action failed; outcome [%d], %s",
                          result.outcome, result.message)
            self.result = False
