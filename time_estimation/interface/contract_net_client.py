import rospy
import actionlib
from mbf_msgs.msg import MoveBaseResult, ExePathGoal, ExePathAction, GetPathAction, GetPathGoal, GetPathActionResult
from nav_msgs.msg import Path
from std_msgs.msg import Float64, String
from geometry_msgs.msg import PoseStamped
from time_estimator import ContractNetTimeEstimator

class ContractNetClient:
    def __init__(self):
        rospy.init_node("contract_net_client")
        self.name_id = "Toru1"
        self.is_busy = False
        self.response = "None"
        self.time_estimator = ContractNetTimeEstimator()
        self.get_path_ac = actionlib.SimpleActionClient("/navigation/move_base_flex/get_path", GetPathAction)
        self.publisher = rospy.Publisher("/multi_robots/propose", Float64, queue_size=50)
        self.cfg_subscriber = rospy.Subscriber("/multi_robots/cfg", PoseStamped, self.propose_cb, queue_size=1)
        self.response_subscriber = rospy.Subscriber("/multi_robots/response", String, self.responses_cb, queue_size=2)
        self.get_path_ac.wait_for_server(rospy.Duration(5))
        self.goal_pose = PoseStamped()
        self.path = Path()
        rospy.spin()

    def responses_cb(self,msg):
        print msg.data
        if msg.data == self.name_id:
            rospy.loginfo("Proposal has been accepted")
            self.is_busy = True

    def propose_cb(self, msg):
        if self.is_busy:
            rospy.loginfo("Ignoring CFG")
            return
        rospy.loginfo("CFP received from server")
        self.goal_pose = msg
        self.get_path()

    def get_path(self):
        rospy.loginfo("Sending To get_path server")
        goal = GetPathGoal(use_start_pose=False,
                           target_pose=self.goal_pose)
        self.get_path_ac.send_goal(goal,done_cb = self.plan_done_cb)
        rospy.loginfo("Calling move_base_flex/get_path")
        self.get_path_ac.wait_for_result(timeout=rospy.Duration(10))
        rospy.loginfo("get_path is done")
        propose = self.time_estimator.calculate_time(self.path)
        self.publisher.publish(Float64(data=propose))
        #TODO msg
        rospy.loginfo("Waiting for the response")

    def plan_done_cb(self,status, result):
        if result.outcome == MoveBaseResult.SUCCESS:
            self.path = result.path
            rospy.loginfo("Get path action succeeded")
        else:
            rospy.logerr("Get path action failed; outcome [%d], %s",
                          result.outcome, result.message)


class ExePathClass:
    def __init__(self):
        self.exe_path_ac = actionlib.SimpleActionClient("/navigation/move_base_flex/exe_path", ExePathAction)
        self.exe_path_ac.wait_for_server(rospy.Duration(5))
        self.result = False

    def execute(self, request_path):
        goal = ExePathGoal(path=request_path)
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
