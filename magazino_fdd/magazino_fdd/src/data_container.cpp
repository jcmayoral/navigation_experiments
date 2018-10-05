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
    last_time_ = ros::Time::now();
}

void DataContainer::updateData(double new_data, int index){
    //std::cout << index <<  "," << data_.size() << std::endl;
    data_[index].push_front(new_data);
    
    if (data_[index].size() > window_size_){
        data_[index].pop_back();
    }
}

void DataContainer::reset(){
    for (int i=0; i<samples_number_;++i)
        data_[i].clear();
}

DataContainer::DataContainer(const std::string id, bool required_statistics, int samples_number, int window_size, double max_delay, double max_diff): last_time_(ros::Time::now()),
        window_size_(window_size), max_delay_(max_delay), data_id_(std::string(id)),
        is_signal_delayed_(false), samples_number_(samples_number), max_diff_(max_diff)
{
    //std::strcpy(data_id_, id);
    if (required_statistics){
        check = std::bind(&DataContainer::statistics_check,this);
    }
    else{
        check = std::bind(&DataContainer::default_check,this);
    }
    
    //TODO
    data_.resize(samples_number);
    window_mean_.resize(samples_number);
    window_std_.resize(samples_number);
    last_window_std_.resize(samples_number);
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
    bool result = false;
    
    if (data_.size() < 2){
        ROS_INFO_ONCE("Waiting for data");
        return false;
    } 

    std::vector<std::list<double>>::iterator it = data_.begin();
    double new_mean; 
    double last_value;
    std::list<double> inner_list;
 
    for (; it!= data_.end(); ++it){
        new_mean = std::accumulate(it->begin(), it->end(), 0.0)/it->size();
        window_mean_[std::distance(data_.begin(), it)] = new_mean;
        window_std_[std::distance(data_.begin(), it)] = std::sqrt(variance(*it, new_mean));
        inner_list = *it;
        if (inner_list.size() < 1)
            break;
        
        last_value = inner_list.front();
        
        for (std::list<double>::iterator inner_it = inner_list.begin();inner_it!=inner_list.end(); ++inner_it){
            if (fabs(last_value - *inner_it) > max_diff_){
                ROS_WARN_STREAM("Anomaly detected on index "<< std::distance(data_.begin(), it));
                ROS_WARN_STREAM("Maximum difference between values  "<< max_diff_ << " occurred on " << data_id_);
                result = true;
            }
            last_value = *inner_it;
        }
    }
    double delay = double((ros::Time::now()-last_time_).toSec());
    is_signal_delayed_ = false;
    //std::cout << "TIMER" << delay << "," << data_id_ <<std::endl;

    
    if (delay > max_delay_){
        is_signal_delayed_ = true;
        ROS_WARN_STREAM("Signal delayed in "<< data_id_ << " by " <<delay << " seconds");
        result = true;
        //return true;
    }
    
    //std::cout << "DIFF on " << data_id_ << " is "<< delay_ << std::endl;
    for (int i=0; i< samples_number_; ++i){
        if (fabs(window_std_[i] - last_window_std_[i])> 0.4){
            ROS_WARN_STREAM("Anomaly detected on  index " << i);
            ROS_WARN_STREAM("Anomaly detected on  " << data_id_);// << " with rate " << fabs(window_std_/last_window_std_));
            result = true;
        }
        last_window_std_[i] = window_std_[i];
    }
    
    //lock.unlock();
    //mtx->unlock();
    //ROS_INFO_THROTTLE(5, "ALL GOOD");
    last_window_std_ = window_std_;
    return result;
}

DataContainer::~DataContainer() {
}

