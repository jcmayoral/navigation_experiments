#!/usr/bin/env python
import sys
import os
import tf
import copy
import yaml
import numpy as np

import rospy
import actionlib

from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Path

from mag_common_py_libs.geometry import quaternion_from_yaw, pose2str
from dynamic_reconfigure.client import Client, DynamicReconfigureCallbackException
from move_base_flex_msgs.msg import MoveBaseResult, ExePathGoal, ExePathAction, GetPathAction, GetPathGoal, GetPathActionResult
from utilities.execution_tools import PathConstructor, ExecutionAnalyzer
from utilities.configuration import ConfigurationManager, ResultSaver

__author__ = 'banos'

class GetPathClass:
    def __init__(self):
        self.get_path_ac = actionlib.SimpleActionClient("/navigation/move_base_flex/get_path", GetPathAction)
        self.get_path_ac.wait_for_server(rospy.Duration(5))
        self.path = Path()

    def execute(self, start, goal):
        t0 = rospy.get_time()
        goal = GetPathGoal(use_start_pose=True,
                           start_pose=start,
                           target_pose=goal)
        self.get_path_ac.send_goal(goal,done_cb = self.plan_done_cb)
        rospy.loginfo("Calling move_base_flex/get_path")
        self.get_path_ac.wait_for_result(timeout=rospy.Duration(10))

    def plan_done_cb(self,status, result):
        t1 = rospy.get_time()
        if result.status == MoveBaseResult.SUCCESS:
            self.path = result.path
            rospy.loginfo("Get path action succeeded")
        else:
            rospy.logerr("Get path action failed; status [%d], error code [%d]: %s",
                          result.status, result.error_code, result.error_msg)

    def get_path(self):
        return self.path

class ExePathClass:
    def __init__(self):
        self.exe_path_ac = actionlib.SimpleActionClient("/navigation/move_base_flex/exe_path", ExePathAction)
        self.exe_path_ac.wait_for_server(rospy.Duration(5))
        self.result = False

    def execute(self, request_path):
        goal = ExePathGoal(path=request_path)
        self.exe_path_ac.send_goal(goal, done_cb=self.execution_done_cb)
        self.exe_path_ac.wait_for_result(timeout=rospy.Duration(120))
        return self.result

    def execution_done_cb(self,status, result):
        if result.status == MoveBaseResult.SUCCESS:
            rospy.loginfo("Exec path action succeeded")
            self.result = True
        else:
            rospy.logerr("Exec path action failed; status [%d], error code [%d]: %s",
                          result.status, result.error_code, result.error_msg)
            self.result = False

def get_robot_pose():
    listener = tf.TransformListener()
    listener.waitForTransform("map", "base_link", rospy.Time(0), rospy.Duration(.5))
    position, o_quaternion = listener.lookupTransform("map", "base_link", rospy.Time(0))

    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "map"
    pose.pose.position.x = position[0]
    pose.pose.position.y = position[1]
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(o_quaternion)
    quaternion = tf.transformations.quaternion_from_euler(0,0, yaw)
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]
    return pose

def get_pose(poses, goal_id):
    desired_goal = poses[goal_id]
    _pose = PoseStamped()
    _pose.header.stamp = rospy.Time.now()
    _pose.header.frame_id = "map"
    _pose.pose.position.x = desired_goal['x']
    _pose.pose.position.y = desired_goal['y']
    _pose.pose.orientation = quaternion_from_yaw(desired_goal['yaw'])
    return _pose

def get_path(path_getter, start_pose, goal_pose):
    path_getter.execute(start_pose, goal_pose)
    return path_getter.get_path()

def execute_path(path_constructor, path_executter, execution_analyzer, path):
    path_constructor.reset()
    execution_analyzer.reset()
    return path_executter.execute(path)

def calculate_curvature(path):
    listener = tf.TransformListener()
    lenght = len(path.poses)
    listener.waitForTransform(path.header.frame_id, "base_link", rospy.Time(0),rospy.Duration(1))

    p0 = path.poses[0]
    quat = [path.poses[0].pose.orientation.x, path.poses[0].pose.orientation.y, path.poses[0].pose.orientation.z, path.poses[0].pose.orientation.w]
    yaw0 = tf.transformations.euler_from_quaternion(quat)[2]

    dx = list()
    dy = list()
    dz = list()
    ddx = list()
    ddy = list()
    ddz = list()

    if len(path.poses) < 2:
        print "ERROR IN PATH"
        return np.array([0,0,0,0,0,0])

    for p in path.poses[1:]:
        dx.append(p.pose.position.x - p0.pose.position.x)
        dy.append(p.pose.position.y - p0.pose.position.y)
        quat = [p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w]
        yaw = tf.transformations.euler_from_quaternion(quat)[2]
        dz.append(yaw - yaw0)
        p0 = p
        yaw0 = yaw


    dx0 = dx[0]
    dy0 = dy[0]
    dz0 = dz[0]

    for x,y,z in zip(dx[1:],dy[1:],dz[1:]):
        ddx.append(x - dx0)
        ddy.append(y - dy0)
        ddz.append(z - dz0)
        dx0 = x
        dy0 = y
        dz0 = z

    dx = np.sum(dx,axis=0)
    dy = np.sum(dy,axis=0)
    dz = np.sum(dz,axis=0)
    ddx = np.sum(ddx, axis=0)
    ddy = np.sum(ddy,axis=0)
    ddz = np.sum(ddz,axis=0)
    #K = float(ddy * dx - ddx * dy) / float(np.power(dx, 2.) + np.power(dy, 2))
    return np.array([dx,dy,dz,ddx,ddy,ddz])

def update_configuration(dynamic_reconfiguration_client, new_config):
    rospy.loginfo(new_config)
    try:
        dynamic_reconfiguration_client.update_configuration(new_config)
    except DynamicReconfigureCallbackException:
        rospy.logerr("Something goes wrong")

def execute_cycle(path_constructor, path_executter, execution_analyzer, start_to_goal_path_curvature, goal_to_start_path_curvature, results):
    result = execute_path(path_constructor, path_executter, execution_analyzer, start_to_goal_path)

    if result:
        results["start_to_goal_curvature"] = calculate_curvature(path_constructor.get_path())-start_to_goal_path_curvature
        rospy.logwarn("Start to Goal Curvature Difference " + str(results["start_to_goal_curvature"]))
        results["start_accumulated_ang_error"] = execution_analyzer.get_accumulated_error()
        rospy.logwarn("accumulate_error" + str(results["start_accumulated_ang_error"]))
    else:
        return False

    result = execute_path(path_constructor, path_executter, execution_analyzer, goal_to_start_path)

    if result:
        results["goal_to_start_curvature"] = calculate_curvature(path_constructor.get_path())-goal_to_start_path_curvature
        rospy.logwarn("Goal to Start Curvature Difference " + str(results["goal_to_start_curvature"]))
        results["goal_accumulated_ang_error"] = execution_analyzer.get_accumulated_error()
        rospy.logwarn("accumulate_error" + str(results['goal_accumulated_ang_error']))
    else:
        return False

    return True

if __name__ == '__main__':

    poses_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'cfg/' + sys.argv[1]+".yaml")
    file_stream = file(poses_file, 'r')
    poses = yaml.load(file_stream)

    rospy.init_node("get_exe_path")

    dyn_client = Client("/navigation/move_base_flex/OrientedDWAPlanner", None)
    path_getter = GetPathClass()
    path_executter = ExePathClass()
    path_constructor = PathConstructor()
    execution_analyzer = ExecutionAnalyzer()
    configuration_manager = ConfigurationManager()
    result_saver = ResultSaver()

    robot_pose = get_robot_pose()
    start_pose = get_pose(poses, 'start_pose')
    goal_pose = get_pose(poses, 'goal_pose')

    robot_to_start_path = get_path(path_getter, robot_pose, start_pose)
    execute_path(path_constructor, path_executter, execution_analyzer, robot_to_start_path)

    start_to_goal_path = get_path(path_getter, start_pose, goal_pose)
    start_to_goal_path_curvature = calculate_curvature(start_to_goal_path)
    rospy.logwarn("Start to Goal Expected Curvature " + str(start_to_goal_path_curvature))
    goal_to_start_path = get_path(path_getter, goal_pose, start_pose)
    goal_to_start_path_curvature = calculate_curvature(goal_to_start_path)
    rospy.logwarn("Goal to Start Expected Curvature " + str(goal_to_start_path_curvature))

    planner_iteration = False

    new_planners_configurations = dict()
    cycles_number = int(sys.argv[2])
    execution_result = True

    for i in range(cycles_number):
        new_params = dict()
        new_results = dict()
        rospy.loginfo("CYCLE %d of %d", i, cycles_number)

        if not execution_result:
            rospy.logerr("Error in Execution")
            rospy.logerr("RESET PARAMS")
            configuration_manager.restart_params()

        configuration_manager.get_new_param_values(new_params)
        print new_params
        update_configuration(dyn_client, new_params)
        execution_result = execute_cycle(path_constructor, path_executter, execution_analyzer, start_to_goal_path_curvature, goal_to_start_path_curvature, new_results)
        result_saver.save_results(new_params, new_results)
