#!/usr/bin/env python
import sys
import os
import yaml
import rospy

from nav_msgs.msg import Path
from dynamic_reconfigure.client import Client, DynamicReconfigureCallbackException
from utilities.execution_tools import PathConstructor, ExecutionAnalyzer
from utilities.configuration import ConfigurationManager, ResultSaver, TestSample
from utilities.util_functions import get_pose, get_path, get_robot_pose, calculate_curvature, fake_path
from utilities.data_analysis import analyze_data, get_best_configuration
from interfaces.move_base_flex_interfaces import GetPathClass, ExePathClass

__author__ = 'banos'

class AutomaticTestExecution:
    def __init__(self, environment='fake', number_cycles=3, distance = 0, step=0):
        rospy.init_node("banos_experimental_manager")
        self.number_cycles = number_cycles
        self.dyn_client = Client("/navigation/move_base_flex/OrientedDWAPlanner", None)
        self.path_getter = GetPathClass()
        self.path_executter = ExePathClass()
        self.path_constructor = PathConstructor()
        self.execution_analyzer = ExecutionAnalyzer()
        self.configuration_manager = ConfigurationManager()
        self.result_saver = ResultSaver(environment)
        self.required_distance = distance
        self.required_step = step
        self.paths = dict()
        self.poses_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'cfg/' + environment +".yaml")
        self.path_publisher = rospy.Publisher("my_path", Path)

    def init(self, curve_type = "straigth"):
        robot_pose = get_robot_pose()
        try:
            file_stream = file(self.poses_file, 'r')
            self.poses = yaml.load(file_stream)
            self.start_pose = get_pose(poses, 'start_pose')
            self.goal_pose = get_pose(poses, 'goal_pose')
            robot_to_start_path = get_path(self.path_getter, robot_pose, self.start_pose)
            self.paths["start_to_goal_"] = get_path(self.path_getter, self.start_pose, self.goal_pose)
            self.paths["goal_to_start_"] = get_path(self.path_getter, self.goal_pose, self.start_pose)
            rospy.loginfo("Sending the robot to start pose")
            self.execute_path(robot_to_start_path)

        except:
            rospy.logwarn("Poses Config File not selected... Faking Paths")
            [self.start_pose, self.goal_pose, self.paths["start_to_goal_"], self.paths["goal_to_start_"]] = fake_path(distance=self.required_distance, step=self.required_step,curve_type=curve_type)

    def run_tests(self):
        execution_result = True

        for i in range(1,self.number_cycles):
            new_params = dict()
            new_results = dict()
            rospy.loginfo("Cycle %d of %d", i, self.number_cycles)

            if not execution_result:
                rospy.logerr("Error in Execution")
                rospy.logerr("Set Default Parameters and sending the robot to Start Pose")
                self.configuration_manager.restart_params()
                robot_pose = get_robot_pose()
                robot_to_start_path = get_path(self.path_getter, robot_pose, self.start_pose)
                self.path_executter.execute(robot_to_start_path)

            self.configuration_manager.get_new_param_values(new_params)
            self.update_configuration(new_params)
            execution_result = self.execute_cycle(new_results)
            self.result_saver.save_results(new_params, new_results)

    def execute_path(self, path):
        self.path_constructor.reset()
        self.execution_analyzer.reset()
        return self.path_executter.execute(path)

    def update_configuration(self,new_config):
        rospy.loginfo("Updating Configuration")
        try:
            self.dyn_client.update_configuration(new_config)
        except DynamicReconfigureCallbackException:
            rospy.logerr("Something goes wrong")

    def execute_cycle(self,results):
        for key, path in self.paths.iteritems():
            self.path_publisher.publish(path)
            start_time = rospy.Time.now()
            result = self.execute_path(path)
            end_time = rospy.Time.now()
            if result:
                data = calculate_curvature(self.path_constructor.get_path())
                results[key + "curvature"] = TestSample(lenght=len(data), data = data).get_dict()
                data = self.execution_analyzer.get_accumulated_error()
                results[key + "accumulated_error"] = TestSample(lenght=len(data), data = data).get_dict()
                data = self.execution_analyzer.get_accumulated_velocities()
                results[key + "accumulated_velocities"] = TestSample(lenght=len(data), data = data).get_dict()
                results[key + "exection_time"] = TestSample(lenght=1, data = (end_time - start_time).to_sec()).get_dict()
            else:
                return False

        return True

if __name__ == '__main__':
    automatic_tuning = AutomaticTestExecution(distance = 1.0, step=0.1)
    automatic_tuning.init(curve_type='left_curve')
    automatic_tuning.run_tests()
    #analyze_data()
    #get_best_configuration()
