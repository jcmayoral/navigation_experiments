/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   magazino_fdd_node.cpp
 * Author: banos
 *
 * Created on September 26, 2018, 10:59 AM
 */

#include <ros/ros.h>
#include <magazino_fdd/monitor.h>
#include <topic_tools/shape_shifter.h>
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "magazino_fault_detection");
  ros::NodeHandle node;

  monitor main_monitor;
}
