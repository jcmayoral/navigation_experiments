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

void monitor::empty_cb(const std_msgs::EmptyConstPtr msg, int index){
    data_containers_[index].updateData(0);
    data_containers_[index].updateTime();
    
}

void monitor::twist_cb(const geometry_msgs::TwistConstPtr msg, int index){
    data_containers_[index].updateTime();
    data_containers_[index].updateData(msg->linear.x);
    data_containers_[index].updateData(msg->angular.z);
}   

void monitor::odom_cb(const nav_msgs::OdometryConstPtr msg, int index){
    data_containers_[index].updateTime();
    data_containers_[index].updateData(msg->twist.twist.linear.x);
    data_containers_[index].updateData(msg->twist.twist.angular.z);
}
    

void monitor::in_cb(const topic_tools::ShapeShifter::ConstPtr& msg, int index, std::string topic_name){
    const std::string& datatype   = msg->getDataType();
    //const std::string& definition = msg->getMessageDefinition();
    //ROS_INFO_STREAM(datatype);
    //ROS_INFO_STREAM(definition);
    if (datatype.compare("std_msgs/Empty") == 0){
        //ROS_INFO("EMPTY MESSAGE");
        main_subscriber_[index].shutdown();
        boost::function<void(const std_msgs::Empty::ConstPtr&) > callback;
        callback = boost::bind( &monitor::empty_cb, this, _1, index) ;
        main_subscriber_[index] = node.subscribe(topic_name, 1, callback);     
    }
    
    if (datatype.compare("geometry_msgs/Twist") == 0){        
        //ROS_INFO_STREAM("Twist MESSAGE index "<< index);
        main_subscriber_[index].shutdown();
        boost::function<void(const geometry_msgs::Twist::ConstPtr&) > callback;
        callback = boost::bind( &monitor::twist_cb, this, _1, index) ;
        main_subscriber_[index] = node.subscribe(topic_name, 1, callback);      
    }

    if (datatype.compare("nav_msgs/Odometry") == 0){        
        //ROS_INFO_STREAM("ODOM MESSAGE index "<< index);
        main_subscriber_[index].shutdown();
        boost::function<void(const nav_msgs::Odometry::ConstPtr&) > callback;
        callback = boost::bind( &monitor::odom_cb, this, _1, index) ;
        main_subscriber_[index] = node.subscribe(topic_name, 1, callback);      
    }
    
    ROS_INFO("Monitor started");
    //ROS_INFO_STREAM(msg->getMessageDefinition());
    //ROS_INFO_STREAM(main_subscriber_.size());
}

monitor::monitor(std::string config_file) {  
    ROS_INFO("Constructor Monitor");
    bool statistics_flags = true;

   std::string path = ros::package::getPath("magazino_fdd");
   //std::cout << path+config_file;
   YAML::Node config_yaml = YAML::LoadFile((path+"/"+config_file).c_str());
   const YAML::Node& topic_names = config_yaml["topics"];
   int id = 0;
  //for (YAML::const_iterator it=topic_names.begin();it!=topic_names.end();++it) {
   for (int i=0; i< topic_names.size(); ++i){
      std::string name = topic_names[i].as<std::string>();
      ROS_INFO_STREAM("Signal to monitor "<< name);
      boost::function<void(const topic_tools::ShapeShifter::ConstPtr&) > callback;
      callback = boost::bind( &monitor::in_cb, this, _1, id, name) ;
      //magazino_fdd::DataContainer container(name,statistics_flags);
      main_subscriber_.push_back( ros::Subscriber(node.subscribe(name, 10, callback)));
      data_containers_.emplace_back(name, statistics_flags);
      //data_containers_.push_back(container);
      ++id;
      //statistics_flags = !statistics_flags;
  }
   
  const YAML::Node& nodes = config_yaml["nodes"];
   for (YAML::const_iterator a= nodes.begin(); a != nodes.end(); ++a){
       std::string n = a->first.as<std::string>();
       //std::string v = a->second.as<std::string>();
       std::list<std::string> v = a->second.as<std::list<std::string> >();
       //std::cout << n << v <<std::endl;
       for (auto i = v.begin(); i!= v.end(); ++i){
           std::cout << *i << std::endl;
       }
   }

   
   
   
   ros::NodeHandle nh;
   timer_ = nh.createTimer(ros::Duration(0.1), &monitor::print_results,this);   
   ros::spin();
}

monitor::monitor(const monitor& orig) {
}

monitor::~monitor() {
}

void monitor::print_results(const ros::TimerEvent&){
    for (std::vector<magazino_fdd::DataContainer>::iterator it=data_containers_.begin(); it != data_containers_.end(); ++it){
        //std::lock_guard<std::mutex> lk(it->mtx_);
        //ROS_INFO_STREAM("Check Monitor " << it->getId());
        if (it->check()){
            ROS_INFO_STREAM("Something Failed on observer " << it->getId());
        }
        it->reset();
        //it->unlock();
    }
}

