import rospy
import actionlib
from mbf_msgs.msg import MoveBaseResult, ExePathGoal, ExePathAction, GetPathAction, GetPathGoal, GetPathActionResult
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, String, Bool

class ContractNetServer:
    def __init__(self):
        rospy.init_node("contract_net_server")
        cfg_publisher = rospy.Publisher("/multi_robots/cfg", PoseStamped, queue_size=1)
        response_publisher = rospy.Publisher("/multi_robots/response", String, queue_size=1)

        #TODO add id from goals
        self.proposes = list()
        rospy.loginfo("Subscribing to Proposals")
        self.subscriber = rospy.Subscriber("/multi_robots/propose", Float64, self.propose_cb)
        rospy.loginfo("Waiting for request")
        task = rospy.wait_for_message("/multi_robots/request", PoseStamped)
        rospy.loginfo("Received Request -> Calling For Proposal")
        cfg_publisher.publish(task)
        rospy.sleep(2.0)

        #TODO msg str and proposes
        if len(self.proposes) > 0:
            min_proposal = min(self.proposes)
            #TODO Attach id of the publisher to the message
            response_publisher.publish(String(data="Toru1"))
            inform = rospy.wait_for_message("/multi_robots/inform", Bool)
            if inform.data:
                rospy.loginfo("Task Succedeed")
            else:
                rospy.logerr("Task Failed")
        else:
            rospy.logerr("Proposes not received")

        self.subscriber.unregister()

    def propose_cb(self, msg):
        rospy.loginfo("Proposal with value "+ str(msg.data))
        self.proposes.append(msg.data)
