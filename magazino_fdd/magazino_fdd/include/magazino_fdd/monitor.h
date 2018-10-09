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
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <string>
#include <mutex>
#include <fstream>

#ifndef MAINMONITOR_H
#define MAINMONITOR_H

class MainMonitor {
public:

    //TODO IDEA Taken from Cartographer
    /*
    template <typename MessageType>
     ::ros::Subscriber SubscribeWithHandler(
                            void (MainMonitor * handler) (int, const std::string&, const typename MessageType::ConstPtr&),
                            const int trajectory_id, 
                            const std::string& topic,
                            ::ros::NodeHandle* const node_handle, 
                            ros::NodeHandle* const node) {
        
            return node_handle->subscribe<MessageType>(
                                        topic, 5,
                                        boost::function<void(const typename MessageType::ConstPtr&)>(
                                                    [node, handler, trajectory_id, topic] (const typename MessageType::ConstPtr& msg) 
                                                    {            
                                                        (node->*handler)(trajectory_id, topic, msg); 
                                                    }));
    }
    */

    MainMonitor(std::string config_file="config/default_config.yml");
    MainMonitor(const MainMonitor& orig);
    void main_cb();
    void in_cb(const topic_tools::ShapeShifter::ConstPtr& msg, int index, std::string topic_name);
    virtual ~MainMonitor();
    // TODO CREATE TEMPLATE
    void empty_cb(const std_msgs::EmptyConstPtr msg, int index);
    void map_cb(const nav_msgs::OccupancyGridConstPtr msg, int index);
    void twist_cb(const geometry_msgs::TwistConstPtr msg, int index);
    void odom_cb(const nav_msgs::OdometryConstPtr msg, int index);
    void imu_cb(const sensor_msgs::ImuConstPtr msg, int index);
    void joints_cb(const sensor_msgs::JointStateConstPtr msg, int index);
    void print_results(const ros::TimerEvent&);
    void isolate_components(std::list<std::string> error_topics);
    double readStatsCPU();

private:
    double getActiveTime(std::vector<double> e);
    std::vector<ros::Subscriber> main_subscriber_;
    std::vector<magazino_fdd::DataContainer> data_containers_;
    ros::NodeHandle node;
    ros::Timer timer_;
    std::string config_file_;
    double last_cpu_usage_;
    double last_cpu_total_;
};

#endif /* MAINMONITOR_H */

