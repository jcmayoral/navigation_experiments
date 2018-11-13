import rospy
import actionlib
from mbf_msgs.msg import MoveBaseResult, ExePathGoal, ExePathAction, GetPathAction, GetPathGoal, GetPathActionResult
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, String, Bool
from multi_robots_comm.msg import ContractNetMsg
import threading

class ContractNetServer(threading.Thread):
    def __init__(self, task):
        threading.Thread.__init__(self)
        self.cfg_publisher = rospy.Publisher("/multi_robots/cfg", PoseStamped, queue_size=1)
        self.response_publisher = rospy.Publisher("/multi_robots/response", ContractNetMsg, queue_size=1)
        rospy.loginfo("Subscribing to Proposals")
        self.subscriber = rospy.Subscriber("/multi_robots/propose", ContractNetMsg, self.propose_cb)

        #TODO add id from goals
        self.proposes = list()

        rospy.loginfo("Received Request -> Calling For Proposal")
        self.cfg_publisher.publish(task)
        rospy.sleep(5.0)

    def run(self):
        #TODO msg str and proposes
        if len(self.proposes) > 0:
            min_proposal = min(self.proposes)
            #TODO Attach id of the publisher to the message
            self.response_publisher.publish(min_proposal)
            inform = rospy.wait_for_message("/multi_robots/inform", Bool)
            if inform.data:
                rospy.loginfo("Task Succedeed")
            else:
                rospy.logerr("Task Failed")
        else:
            rospy.logerr("Proposes not received")

        self.subscriber.unregister()

    def propose_cb(self, msg):
        rospy.loginfo("Proposal from " + msg.robot_id + " with value "+ str(msg.propose_value))
        self.proposes.append(msg)
