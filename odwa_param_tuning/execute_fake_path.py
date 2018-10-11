#!/usr/bin/env python
import sys
import os
import yaml
import rospy

from dynamic_reconfigure.client import Client, DynamicReconfigureCallbackException
from utilities.execution_tools import PathConstructor, ExecutionAnalyzer
from utilities.configuration import ConfigurationManager, ResultSaver, TestSample
from utilities.util_functions import get_pose, get_path, get_robot_pose, calculate_curvature, fake_path
from interfaces.move_base_flex_interfaces import GetPathClass, ExePathClass

__author__ = 'banos'

def execute_path(path_constructor, path_executter, path):
    path_constructor.reset()
    return path_executter.execute(path)


def execute_cycle(path_constructor, path_executter, start_to_goal_path, goal_to_start_path):
    start_time = rospy.Time.now()
    result = execute_path(path_constructor, path_executter, start_to_goal_path)
    end_time = rospy.Time.now()

    start_time = rospy.Time.now()
    result = execute_path(path_constructor, path_executter, goal_to_start_path)
    end_time = rospy.Time.now()

    return True

if __name__ == '__main__':

    rospy.init_node("banos_experimental_manager")

    path_executter = ExePathClass()
    path_constructor = PathConstructor()
    result_saver = ResultSaver()

    rospy.logwarn("Fake path")
    [start_pose, goal_pose, start_to_goal_path, goal_to_start_path] = fake_path(distance=10)

    cycles_number = int(sys.argv[1]) if sys.argv[1] else 10
    execution_result = True

    for i in range(cycles_number):
        rospy.loginfo("Cycle %d of %d", i, cycles_number)
        execution_result = execute_cycle(path_constructor, path_executter, start_to_goal_path, goal_to_start_path)
