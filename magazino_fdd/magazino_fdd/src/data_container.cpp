/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   data_container.cpp
 * Author: banos
 * 
 * Created on September 26, 2018, 2:00 PM
 */

#include <magazino_fdd/data_container.h>
#include <boost/thread/pthread/recursive_mutex.hpp>

using namespace magazino_fdd;

void DataContainer::updateTime(){
    //mtx->lock();
    last_time_ = ros::Time::now();   
    //mtx->unlock();
}

void DataContainer::updateData(double new_data){
    //mtx->lock();
    data_.push_front(new_data);
    
    if (data_.size() > window_size_){
        data_.pop_back();
    }
    //mtx->unlock();
}

DataContainer::DataContainer(std::string id, bool required_statistics): last_time_(ros::Time::now()),
        window_size_(10), window_mean_(0.0), window_std_(0.0), max_delay_(1.0),
        data_id_(id)
{
  
    if (required_statistics){
        check = std::bind(&DataContainer::statistics_check,this);
    }
    else{
        check = std::bind(&DataContainer::default_check,this);
    }
            
}

std::string DataContainer::getId(){
    return data_id_;
}


bool DataContainer::default_check(){
    return false;
}

bool DataContainer::statistics_check(){
    //boost::mutex::scoped_lock lock(*mtx);
    ros::Duration diff = ros::Time::now() - last_time_;
    if (diff.toSec() > max_delay_){
        ROS_WARN_THROTTLE(5,("Monitor delayed " + getId()).c_str() );
        //mtx->unlock();
        return true;
    }
    
    window_mean_ = std::accumulate(data_.begin(), data_.end(), 0.0)/data_.size();
    window_std_ = std::sqrt(variance(data_, window_mean_));
    
    if (window_std_ == 0.0){
        ROS_WARN_THROTTLE(5, ("Monitor frozen " + getId()).c_str() );
        //mtx->unlock();
        return true;
    }
    //lock.unlock();
    //mtx->unlock();
    ROS_INFO_THROTTLE(5, "ALL GOOD");
    return false;
}

DataContainer::~DataContainer() {
}

