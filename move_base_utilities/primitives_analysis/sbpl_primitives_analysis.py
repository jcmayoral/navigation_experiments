import rospy
from visualization_msgs.msg import MarkerArray


class SBPLPrimitiveAnalysis:
    def __init__(self):
        rospy.init_node("sbpl_primitive_analysis")
        self.results = dict()
        rospy.Subscriber("/navigation/move_base_flex/SBPLLatticePlanner/plan_marker", MarkerArray, self.primitives_cb)
        rospy.spin()

    def primitives_cb(self, msg):
        for marker in msg.markers:
            if marker.text in self.results:
                self.results[marker.text]+=1
            else:
                self.results[marker.text]=1
        #self.reset()

    def reset(self):
        rospy.loginfo("Restart")
        self.results = dict()

    def print_results(self):
        for keys,values in self.results.items():
            print(keys, ":", values)

primitives_analyzer = SBPLPrimitiveAnalysis()
primitives_analyzer.print_results()
