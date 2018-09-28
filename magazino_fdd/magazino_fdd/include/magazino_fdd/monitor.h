/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   monitor.h
 * Author: banos
 *
 * Created on September 26, 2018, 10:45 AM
 */

#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <magazino_fdd/data_container.h>
#include <nav_msgs/Odometry.h>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <string>
#include <mutex>

#ifndef MONITOR_H
#define MONITOR_H

class monitor {
public:
    monitor(std::string config_file="config/default_config.yml");
    monitor(const monitor& orig);
    void main_cb();
    void in_cb(const topic_tools::ShapeShifter::ConstPtr& msg, int index, std::string topic_name);
    virtual ~monitor();
    void empty_cb(const std_msgs::EmptyConstPtr msg, int index);
    void twist_cb(const geometry_msgs::TwistConstPtr msg, int index);
    void odom_cb(const nav_msgs::OdometryConstPtr msg, int index);
    void print_results(const ros::TimerEvent&);
    
private:
    std::vector<ros::Subscriber> main_subscriber_;
    std::vector<magazino_fdd::DataContainer> data_containers_;
    ros::NodeHandle node;
    ros::Timer timer_;
};

#endif /* MONITOR_H */

