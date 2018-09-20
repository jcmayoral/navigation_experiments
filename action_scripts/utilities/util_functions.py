import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import rospy
from mag_common_py_libs.geometry import quaternion_from_yaw
import numpy as np
import copy

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

def calculate_curvature(path):
    lenght = len(path.poses)

    p0 = path.poses[0]
    quat = [path.poses[0].pose.orientation.x, path.poses[0].pose.orientation.y, path.poses[0].pose.orientation.z, path.poses[0].pose.orientation.w]
    yaw0 = tf.transformations.euler_from_quaternion(quat)[2]

    dx = list()
    dy = list()
    dz = list()
    ddx = list()
    ddy = list()
    ddz = list()

    if len(path.poses) < 3:
        rospy.logerr("Path Too Short to calculate curvature")
        return [0,0,0,0,0,0]

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

    #K = float(ddy * dx - ddx * dy) / float(np.power(dx, 2.) + np.power(dy, 2))
    return [sum(dx),sum(dy),sum(dz),sum(ddx),sum(ddy),sum(ddz)]

def fake_path(mode = 'straigth', distance = 5.0, local_frame = "/base_link", step = 0.05):
    listener = tf.TransformListener()
    rospy.sleep(4) #Give Time to the listener to charge the entire tf
    print mode
    start_to_goal_fake_path = Path()
    goal_to_start_fake_path = Path()
    start_to_goal_fake_path.header.frame_id = "map"
    goal_to_start_fake_path.header.frame_id = "map"

    #TODO CURVES
    if mode is 'straigth':
        listener.waitForTransform("/map", local_frame, rospy.Time(0),rospy.Duration(3.0))

        for i in np.arange(0, distance, step):
            new_pose = PoseStamped()
            new_pose.header.frame_id = local_frame
            new_pose.header.stamp = rospy.Time(0)
            new_pose.pose.position.x = i
            new_pose.pose.orientation = quaternion_from_yaw(0.0)
            map_pose = listener.transformPose("map", new_pose)
            start_to_goal_fake_path.poses.append(map_pose)

    #Reverse poses to create goal to start
    reversed_poses = copy.deepcopy(start_to_goal_fake_path.poses)
    reversed_poses.reverse()
    goal_to_start_fake_path.poses = reversed_poses
    start_to_goal_fake_path.header.stamp = rospy.Time.now()
    return [start_to_goal_fake_path.poses[0], goal_to_start_fake_path.poses[0], start_to_goal_fake_path, goal_to_start_fake_path]
