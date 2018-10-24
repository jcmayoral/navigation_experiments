#!/usr/bin/env python
from multi_robots_comm.contract_net_server import ContractNetServer
import rospy


while not rospy.is_shutdown():
    ContractNetServer()
