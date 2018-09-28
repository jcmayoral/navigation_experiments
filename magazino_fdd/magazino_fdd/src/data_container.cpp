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
    //std::lock_guard<std::mutex> lk(mtx_);
    last_time_ = std::clock();
}

void DataContainer::updateData(double new_data){
    //std::lock_guard<std::mutex> lk(mtx_);
    data_.push_front(new_data);
    
    if (data_.size() > window_size_){
        //data_.pop_back();
        data_.clear();
    }
}

DataContainer::DataContainer(const std::string id, bool required_statistics): last_time_(std::clock()),
        window_size_(10), window_mean_(0.0), window_std_(0.0), max_delay_(0.1), data_id_(std::string(id)),
        delay_(0.0)
{
    ROS_INFO_STREAM("DATA CONTAINER NAMED" << data_id_);
    //std::strcpy(data_id_, id);
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
    ROS_ERROR("THIS SHOULD NOT BE PRINTED");
    return false;
}

bool DataContainer::statistics_check(){
    std::lock_guard<std::mutex> lk(mtx_);
       
    if (data_.size() < 2){
        ROS_INFO_ONCE("Waiting for data");
        return false;
    } 
    try 
    {
        //auto end = std::chrono::system_clock::now();
        //std::chrono::duration<double> elapsed_seconds = end- last_time_;
        delay_ = double(std::clock()-last_time_)/CLOCKS_PER_SEC;
        //delay_ = double(elapsed_seconds.count());
        if (delay_> max_delay_){
            ROS_WARN_STREAM("Signal delayed " << delay_<< "in "<< data_id_);
            return true;
        }

    }
    catch(std::runtime_error& ex) {
        ROS_ERROR("Exception: [%s]", ex.what());
        ROS_ERROR("Error with delay");
        return true;
    }
    
    //std::cout << "DIFF on " << data_id_ << " is "<< delay_ << std::endl;
 
    window_mean_ = std::accumulate(data_.begin(), data_.end(), 0.0)/data_.size();
    window_std_ = std::sqrt(variance(data_, window_mean_));
 
    if (window_std_ == 0.0){
        ROS_WARN_STREAM("Signal frozen " << data_id_);
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

