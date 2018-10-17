import rospy
import actionlib
from mbf_msgs.msg import MoveBaseResult, ExePathGoal, ExePathAction, GetPathAction, GetPathGoal, GetPathActionResult
from nav_msgs.msg import Path

class GetPathClass:
    def __init__(self):
        self.get_path_ac = actionlib.SimpleActionClient("/navigation/move_base_flex/get_path", GetPathAction)
        self.get_path_ac.wait_for_server(rospy.Duration(5))
        self.path = Path()

    def execute(self, start, goal):
        t0 = rospy.get_time()
        goal = GetPathGoal(use_start_pose=True,
                           start_pose=start,
                           target_pose=goal)
        self.get_path_ac.send_goal(goal,done_cb = self.plan_done_cb)
        rospy.loginfo("Calling move_base_flex/get_path")
        self.get_path_ac.wait_for_result(timeout=rospy.Duration(10))

    def plan_done_cb(self,status, result):
        t1 = rospy.get_time()
        if result.outcome == MoveBaseResult.SUCCESS:
            self.path = result.path
            rospy.loginfo("Get path action succeeded")
        else:
            rospy.logerr("Get path action failed; outcome [%d], %s",
                          result.outcome, result.message)

    def get_path(self):
        return self.path

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
