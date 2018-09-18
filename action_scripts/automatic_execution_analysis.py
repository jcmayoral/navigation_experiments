#!/usr/bin/env python
import sys
import os
import yaml
import rospy

from dynamic_reconfigure.client import Client, DynamicReconfigureCallbackException
from utilities.execution_tools import PathConstructor, ExecutionAnalyzer
from utilities.configuration import ConfigurationManager, ResultSaver
from utilities.util_functions import get_pose, get_path, get_robot_pose, calculate_curvature, fake_path
from interfaces.move_base_flex_interfaces import GetPathClass, ExePathClass

__author__ = 'banos'

def execute_path(path_constructor, path_executter, execution_analyzer, path):
    path_constructor.reset()
    execution_analyzer.reset()
    return path_executter.execute(path)

def update_configuration(dynamic_reconfiguration_client, new_config):
    rospy.loginfo("Updating Configuration")
    try:
        dynamic_reconfiguration_client.update_configuration(new_config)
    except DynamicReconfigureCallbackException:
        rospy.logerr("Something goes wrong")

def execute_cycle(path_constructor, path_executter, execution_analyzer, results):
    result = execute_path(path_constructor, path_executter, execution_analyzer, start_to_goal_path)

    if result:
        results["start_to_goal_curvature"] = calculate_curvature(path_constructor.get_path())
        results["start_to_goal_accumulated_error"] = execution_analyzer.get_accumulated_error()
        results["start_to_goal_accumulated_velocities"] = execution_analyzer.get_accumulated_velocities()
    else:
        return False

    result = execute_path(path_constructor, path_executter, execution_analyzer, goal_to_start_path)

    if result:
        results["goal_to_start_curvature"] = calculate_curvature(path_constructor.get_path())
        results["goal_to_start_accumulated_error"] = execution_analyzer.get_accumulated_error()
        results["goal_to_start_accumulated_velocities"] = execution_analyzer.get_accumulated_velocities()
    else:
        return False

    return True

if __name__ == '__main__':

    rospy.init_node("banos_experimental_manager")

    dyn_client = Client("/navigation/move_base_flex/OrientedDWAPlanner", None)
    path_getter = GetPathClass()
    path_executter = ExePathClass()
    path_constructor = PathConstructor()
    execution_analyzer = ExecutionAnalyzer()
    configuration_manager = ConfigurationManager()
    result_saver = ResultSaver()

    robot_pose = get_robot_pose()

    try:
        poses_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'cfg/' + sys.argv[1]+".yaml")
        file_stream = file(poses_file, 'r')
        poses = yaml.load(file_stream)
        start_pose = get_pose(poses, 'start_pose')
        goal_pose = get_pose(poses, 'goal_pose')
        robot_to_start_path = get_path(path_getter, robot_pose, start_pose)
        start_to_goal_path = get_path(path_getter, start_pose, goal_pose)
        goal_to_start_path = get_path(path_getter, goal_pose, start_pose)
        rospy.loginfo("Sending the robot to start pose")
        execute_path(path_constructor, path_executter, execution_analyzer, robot_to_start_path)

    except:
        rospy.logwarn("Poses Config File not selected... Faking Paths")
        [start_pose, goal_pose, start_to_goal_path, goal_to_start_path] = fake_path()

    cycles_number = int(sys.argv[2]) if sys.argv[2] else 10
    execution_result = True

    for i in range(cycles_number):
        new_params = dict()
        new_results = dict()
        rospy.loginfo("Cycle %d of %d", i, cycles_number)

        if not execution_result:
            rospy.logerr("Error in Execution")
            rospy.logerr("Set Default Parameters and sending the robot to Start Pose")
            configuration_manager.restart_params()
            robot_pose = get_robot_pose()
            robot_to_start_path = get_path(path_getter, robot_pose, start_pose)
            execute_path(path_constructor, path_executter, execution_analyzer, robot_to_start_path)

        configuration_manager.get_new_param_values(new_params)
        update_configuration(dyn_client, new_params)
        execution_result = execute_cycle(path_constructor, path_executter, execution_analyzer, new_results)
        result_saver.save_results(new_params, new_results)
