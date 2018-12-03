import rospy
from visualization_msgs.msg import MarkerArray


class SBPLPrimitiveAnalysis:
    def __init__(self):
        self.current_results = dict()
        rospy.Subscriber("/navigation/move_base_flex/SBPLLatticePlanner/plan_marker", MarkerArray, self.primitives_cb)

    def primitives_cb(self, msg):
        for marker in msg.markers:
            if marker.text in self.current_results:
                self.current_results[marker.text]+=1
            else:
                self.current_results[marker.text]=1
        #self.reset()

    def get_primitive_count(self, key):
        if str(key) in self.current_results:
            return self.current_results[str(key)]
        else:
            #print "Key %d does not exist " % key
            return 0

    def reset_primitives_count(self):
        rospy.loginfo("Restart")
        self.current_results = dict()

    def print_results(self):
        for keys,values in self.current_results.items():
            print(keys, ":", values)


if __name__ == '__main__':
    rospy.init_node("sbpl_primitive_analysis")
    primitives_analyzer = SBPLPrimitiveAnalysis()
    rospy.spin()
    primitives_analyzer.print_results()
