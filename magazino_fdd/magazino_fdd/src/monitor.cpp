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
}

void monitor::odom_cb(const nav_msgs::OdometryConstPtr msg, int index){
    data_containers_[index].updateTime();
    data_containers_[index].updateData(msg->twist.twist.linear.x);
}

void monitor::in_cb(const topic_tools::ShapeShifter::ConstPtr& msg, int index, std::string topic_name){
    const std::string& datatype   = msg->getDataType();
    const std::string& definition = msg->getMessageDefinition();
    ROS_INFO_STREAM(datatype);
    //ROS_INFO_STREAM(definition);
    if (datatype.compare("std_msgs/Empty") == 0){
        ROS_INFO("EMPTY MESSAGE");
        main_subscriber_[index].shutdown();
        boost::function<void(const std_msgs::Empty::ConstPtr&) > callback;
        callback = boost::bind( &monitor::empty_cb, this, _1, index) ;
        main_subscriber_[index] = node.subscribe(topic_name, 10, callback);     
    }
    
    if (datatype.compare("geometry_msgs/Twist") == 0){        
        ROS_INFO("Twist MESSAGE");
        main_subscriber_[index].shutdown();
        boost::function<void(const geometry_msgs::Twist::ConstPtr&) > callback;
        callback = boost::bind( &monitor::twist_cb, this, _1, index) ;
        main_subscriber_[index] = node.subscribe(topic_name, 10, callback);      
    }


    if (datatype.compare("nav_msgs/Odometry") == 0){        
        ROS_INFO("ODOM MESSAGE");
        main_subscriber_[index].shutdown();
        boost::function<void(const nav_msgs::Odometry::ConstPtr&) > callback;
        callback = boost::bind( &monitor::odom_cb, this, _1, index) ;
        main_subscriber_[index] = node.subscribe(topic_name, 10, callback);      
    }
    //ROS_INFO_STREAM(msg->getMessageDefinition());
   
    //auto new_msg = msg->instantiate<std_msgs::Empty>();
}

// Note, you can recycle this callback and subscribe to multiple topics
void monitor::messageCallback(const topic_tools::ShapeShifter::ConstPtr& msg,
                     const std::string &topic_name )
{
    ROS_INFO("IN");
}

monitor::monitor(std::string config_file) {  
  bool statistics_flags = true;

   std::string path = ros::package::getPath("magazino_fdd");
   //std::cout << path+config_file;
   YAML::Node config_yaml = YAML::LoadFile((path+"/"+config_file).c_str());
   const YAML::Node& topic_names = config_yaml["topics"];
   int id = 0;
  //for (YAML::const_iterator it=topic_names.begin();it!=topic_names.end();++it) {
   for (int i=0; i< topic_names.size(); ++i){
      std::string name = topic_names[i].as<std::string>();
      ROS_INFO_STREAM("MONITORING "<< name);
      boost::function<void(const topic_tools::ShapeShifter::ConstPtr&) > callback;
      callback = boost::bind( &monitor::in_cb, this, _1, id, name) ;
      main_subscriber_.push_back( ros::Subscriber(node.subscribe(name, 10, callback)));
      data_containers_.emplace_back(name, statistics_flags);
      ++id;
      //statistics_flags = !statistics_flags;
  }
  ROS_INFO("!");
  //topic_tools::ShapeShifter my_msg  = *(ros::topic::waitForMessage<topic_tools::ShapeShifter>("b"));
  ROS_INFO("!B");
  //ros::spin();
//    ros::TransportHints g_th;
  //  ros::NodeHandle nh;
    //main_subscriber_ = new ros::Subscriber(nh.subscribe("TEST", 10, &monitor::in_cb, g_th));
}

monitor::monitor(const monitor& orig) {
}

monitor::~monitor() {
}

void monitor::print_results(){
    for (std::vector<magazino_fdd::DataContainer>::iterator it=data_containers_.begin(); it != data_containers_.end(); ++it){
        if (it->check()){
            
            //ROS_INFO_STREAM("ERROR IN "<< it->getId());
        }
    }
}

