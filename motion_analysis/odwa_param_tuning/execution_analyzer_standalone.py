from utilities.execution_tools import ExecutionAnalyzer
import rospy

rospy.init_node("banos_execution_analyzer")

exec_analyzer = ExecutionAnalyzer()

while not rospy.is_shutdown():
    raw_input("Press Enter to Start...")
    exec_analyzer.reset()
    raw_input("Press Enter to Stop...")
    print "ERRORS ", exec_analyzer.get_accumulated_error()
