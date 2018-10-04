/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   monitor.cpp
 * Author: banos
 * 
 * Created on September 26, 2018, 10:45 AM
 */ 

#include <magazino_fdd/monitor.h>

void MainMonitor::empty_cb(const std_msgs::EmptyConstPtr msg, int index){
    data_containers_[index].updateData(0,0);
    data_containers_[index].updateTime();
    
}

void MainMonitor::twist_cb(const geometry_msgs::TwistConstPtr msg, int index){
    data_containers_[index].updateTime();
    data_containers_[index].updateData(msg->linear.x,0);
    data_containers_[index].updateData(msg->angular.z,1);
}   

void MainMonitor::odom_cb(const nav_msgs::OdometryConstPtr msg, int index){
    data_containers_[index].updateTime();
    data_containers_[index].updateData(msg->twist.twist.linear.x,0);
    data_containers_[index].updateData(msg->twist.twist.angular.z,1);
}
    

void MainMonitor::map_cb(const nav_msgs::OccupancyGridConstPtr msg, int index){
    data_containers_[index].updateTime();
    data_containers_[index].updateData(msg->info.origin.position.x,0);
    data_containers_[index].updateData(msg->info.origin.position.y,1);
}

void MainMonitor::imu_cb(const sensor_msgs::ImuConstPtr msg, int index){
    data_containers_[index].updateTime();
    data_containers_[index].updateData(msg->linear_acceleration.x,0);
    data_containers_[index].updateData(msg->angular_velocity.z,1);
}

void MainMonitor::in_cb(const topic_tools::ShapeShifter::ConstPtr& msg, int index, std::string topic_name){
    const std::string& datatype   = msg->getDataType();
    //const std::string& definition = msg->getMessageDefinition();
    //ROS_INFO_STREAM(datatype);
    //ROS_INFO_STREAM(definition);
    if (datatype.compare("std_msgs/Empty") == 0){
        //ROS_INFO("EMPTY MESSAGE");
        main_subscriber_[index].shutdown();
        boost::function<void(const std_msgs::Empty::ConstPtr&) > callback;
        callback = boost::bind( &MainMonitor::empty_cb, this, _1, index) ;
        main_subscriber_[index] = node.subscribe(topic_name, 1, callback);     
    }
    
    if (datatype.compare("geometry_msgs/Twist") == 0){        
        //ROS_INFO_STREAM("Twist MESSAGE index "<< index);
        main_subscriber_[index].shutdown();
        boost::function<void(const geometry_msgs::Twist::ConstPtr&) > callback;
        callback = boost::bind( &MainMonitor::twist_cb, this, _1, index) ;
        main_subscriber_[index] = node.subscribe(topic_name, 1, callback);      
    }

    if (datatype.compare("nav_msgs/Odometry") == 0){        
        //ROS_INFO_STREAM("ODOM MESSAGE index "<< index);
        main_subscriber_[index].shutdown();
        boost::function<void(const nav_msgs::Odometry::ConstPtr&) > callback;
        callback = boost::bind( &MainMonitor::odom_cb, this, _1, index) ;
        main_subscriber_[index] = node.subscribe(topic_name, 1, callback);      
    }

    if (datatype.compare("nav_msgs/OccupancyGrid") == 0){
        main_subscriber_[index].shutdown();
        boost::function<void(const nav_msgs::OccupancyGrid::ConstPtr&) > callback;
        callback = boost::bind( &MainMonitor::map_cb, this, _1, index) ;
        main_subscriber_[index] = node.subscribe(topic_name, 1, callback);      
    }

    if (datatype.compare("sensor_msgs/Imu") == 0){
        main_subscriber_[index].shutdown();
        boost::function<void(const sensor_msgs::Imu::ConstPtr&) > callback;
        callback = boost::bind( &MainMonitor::imu_cb, this, _1, index) ;
        main_subscriber_[index] = node.subscribe(topic_name, 1, callback);      
    }
        
    ROS_INFO("Monitor started");
    //ROS_INFO_STREAM(msg->getMessageDefinition());
    //ROS_INFO_STREAM(main_subscriber_.size());
}

MainMonitor::MainMonitor(std::string config_file) {
    ROS_INFO("Constructor Monitor");
    bool statistics_flags = true;
    config_file_ = config_file;
    std::string path = ros::package::getPath("magazino_fdd");
    YAML::Node config_yaml = YAML::LoadFile((path+"/"+config_file_).c_str());
    
    const YAML::Node& topic_names = config_yaml["topics"];
    int id = 0;
 
    for (int i=0; i< topic_names.size(); ++i){
        std::string name = topic_names[i].as<std::string>();
        ROS_INFO_STREAM("Signal to monitor "<< name);
        boost::function<void(const topic_tools::ShapeShifter::ConstPtr&) > callback;
        callback = boost::bind( &MainMonitor::in_cb, this, _1, id, name) ;
        data_containers_.emplace_back(name, statistics_flags,3);
        main_subscriber_.push_back( ros::Subscriber(node.subscribe(name, 10, callback)));
        //data_containers_.push_back(container);
        ++id;
        //statistics_flags = !statistics_flags;
    }
    ros::NodeHandle nh;
    timer_ = nh.createTimer(ros::Duration(0.5), &MainMonitor::print_results,this);
    ros::spin();
}


void MainMonitor::isolate_components(std::list<std::string> error_topics){
    error_topics.sort();
    std::string path = ros::package::getPath("magazino_fdd");
    YAML::Node config_yaml = YAML::LoadFile((path+"/"+config_file_).c_str());
    const YAML::Node& nodes = config_yaml["nodes"];
    int counter = 0;
    
    ROS_INFO_STREAM(error_topics.size() << " Errors reported");
 
    for (YAML::const_iterator a= nodes.begin(); a != nodes.end(); ++a){
        std::string n = a->first.as<std::string>();
        //std::string v = a->second.as<std::string>();
        std::list<std::string> v = a->second.as<std::list<std::string> >();
        //std::cout << n << v <<std::endl;
        /*
        for (auto i = v.begin(); i!= v.end(); ++i){
           std::cout << *i << std::endl;
        }
        */
        v.sort();
        if (v.size() == error_topics.size()){
            counter = 0;
            for (auto i1 = v.begin(), i2 = error_topics.begin(); i1!= v.end(); ++i1, ++i2){
              if (i1->compare(*i2)==0){
                ++counter;
              }
            }
            
            if (counter == error_topics.size())
                ROS_ERROR_STREAM("ERROR in " << n);
        }
   }
}

MainMonitor::MainMonitor(const MainMonitor& orig) {
}

MainMonitor::~MainMonitor() {
}

void MainMonitor::print_results(const ros::TimerEvent&){
    std::list <std::string> detected_errors;
    for (std::vector<magazino_fdd::DataContainer>::iterator it=data_containers_.begin(); it != data_containers_.end(); ++it){
        //std::lock_guard<std::mutex> lk(it->mtx_);
        //ROS_INFO_STREAM("Check Monitor " << it->getId());
        if (it->check()){
            detected_errors.push_back(it->getId());
        }
        it->reset();
        //it->unlock();
    }
    
    if (detected_errors.size()>0){
        isolate_components(detected_errors);
    }
}

