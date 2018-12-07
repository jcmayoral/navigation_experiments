import rospy
from visualization_msgs.msg import MarkerArray


class SBPLPrimitiveAnalysis:
    def __init__(self):
        self.current_results = dict()
        self.primitive_list = list()
        rospy.Subscriber("/navigation/move_base_flex/SBPLLatticePlanner/plan_marker", MarkerArray, self.primitives_cb)

    def get_primitive_list(self):
        return self.primitive_list

    def primitives_cb(self, msg):
        self.primitive_list = list()

        for marker in msg.markers:
            self.primitive_list.append(marker.text)
            if marker.text in self.current_results:
                if 'rip' in marker.text:
                    self.current_results['rip']+=1
                else:
                    self.current_results[marker.text]+=1
            else:
                if 'rip' in marker.text:
                    self.current_results['rip']=1
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
