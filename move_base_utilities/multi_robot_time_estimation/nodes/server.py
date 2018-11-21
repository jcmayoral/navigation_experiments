#!/usr/bin/env python
from multi_robots_comm.contract_net_server import ContractNetServer
import rospy
from geometry_msgs.msg import PoseStamped
import threading

rospy.init_node("contract_net_server")

current_jobs = list()
while not rospy.is_shutdown():
    rospy.loginfo("Waiting for new request")
    task = rospy.wait_for_message("/multi_robots/request", PoseStamped)
    contractnet = ContractNetServer(task)
    current_jobs.append(threading.Thread(target=contractnet.run).start())
    #new_thread.join()
