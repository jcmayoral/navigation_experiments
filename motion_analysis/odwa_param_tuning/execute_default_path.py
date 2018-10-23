#!/usr/bin/env python
import sys
import os
import yaml
import rospy
import tf

from dynamic_reconfigure.client import Client, DynamicReconfigureCallbackException
from utilities.execution_tools import PathConstructor, ExecutionAnalyzer
from utilities.configuration import ConfigurationManager, ResultSaver, TestSample
from utilities.util_functions import get_pose, get_path, get_robot_pose, calculate_curvature, fake_path
from interfaces.move_base_flex_interfaces import GetPathClass, ExePathClass
from geometry_msgs.msg import PoseStamped
from mag_common_py_libs.geometry import quaternion_from_yaw

__author__ = 'banos'

def execute_path(path_constructor, path_executter, path):
    path_constructor.reset()
    return path_executter.execute(path)

def get_end_pose(distance = 10):
    listener = tf.TransformListener()
    listener.waitForTransform("/map", "base_link", rospy.Time(0),rospy.Duration(3.0))
    new_pose = PoseStamped()

    new_pose.header.frame_id = "base_link"
    new_pose.header.stamp = rospy.Time(0)
    new_pose.pose.position.x = distance
    new_pose.pose.orientation = quaternion_from_yaw(0.0)
    map_pose = listener.transformPose("map", new_pose)
    return map_pose

def execute_motion(path_constructor, path_executter, path):
    result = execute_path(path_constructor, path_executter, path)
    return True

if __name__ == '__main__':

    rospy.init_node("banos_experimental_manager")

    path_lenght = 10.0
    path_getter = GetPathClass()
    path_executter = ExePathClass()
    path_constructor = PathConstructor()

    cycles_number = int(sys.argv[1]) if sys.argv[1] else 10
    execution_result = True
    end_pose = get_end_pose()

    for i in range(cycles_number):
        rospy.loginfo("Cycle %d of %d", i, cycles_number)
        path = get_path(path_getter, get_robot_pose(), get_end_pose(distance=path_lenght))
        execution_result = execute_motion(path_constructor, path_executter, path)
        path = get_path(path_getter, get_robot_pose(), get_end_pose(distance=- path_lenght))
        execution_result = execute_motion(path_constructor, path_executter, path)
