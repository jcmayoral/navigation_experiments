/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   canopen_odom_creator_node.cpp
 * Author: banos
 *
 * Created on November 9, 2018, 2:07 PM
 */

#include <cstdlib>
#include <ros/ros.h>
#include <canopen_core/diffdrive/diffdrive_ros_interface.h>
#include <canopen_core/diffdrive/diffdrive_manager_hack.h>

#include <canopen_core/diffdrive/odometry.h>

using namespace std;

/*
 * 
 */

void my_reset(){
    
}

int main(int argc, char** argv) {
    ros::init(argc,argv,"canopen_odom_creator_node");
    /*
    DiffDriveRosInterface* diff_drive_interface_ = new DiffDriveRosInterface(my_reset);
    diff_drive_interface_->start();
    ros::spin();
    diff_drive_interface_->stop();
    */
    DiffDriveManagerHack diffdrive_manager_;
    diffdrive_manager_.start();
    
    while(ros::ok()){
        ROS_INFO_ONCE("Running");
    }
        
    
    ROS_ERROR("STOPPING");
    //TODO solve some bug on stopping
    diffdrive_manager_.stop();
    return 1;
}

