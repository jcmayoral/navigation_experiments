/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   data_container.h
 * Author: banos
 *
 * Created on September 26, 2018, 1:51 PM
 */

#ifndef DATA_CONTAINER_H
#define DATA_CONTAINER_H

#include <ros/ros.h>
#include <mutex>
#include <string>
//#include <boost/thread/thread.hpp>
//#include <boost/thread/mutex.hpp>
//boost::mutex mtx; 
#include <ctime>
 
namespace magazino_fdd{

    class DataContainer{
        template<typename T>
        T variance(const std::list<T> &li, T mean)
        {
            size_t sz = li.size();
            if (sz == 1)
                return 0.0;
            // Calculate the mean
            //U mean_calc = std::accumulate(li.begin(), li.end(), 0.0) / sz;
            // Now calculate the variance
            auto variance_func = [&mean, &sz](T accumulator, const T& val)
            {
                return accumulator + ((val - mean)*(val - mean) / (sz - 1));
            };
            return std::accumulate(li.begin(), li.end(), 0.0, variance_func);
        }

    public:
        DataContainer(const std::string id, bool required_statistics=true);
        DataContainer(const DataContainer& other)//:data_(other.data_)
        {
            //std::cout << "THIS IS USED BUT WHEN?" <<std::endl;
            this->data_id_ = other.data_id_;
            //this->last_time_ = std::chrono::system_clock::now();
            this->last_time_ =ros::Time::now();
            this->window_size_ = other.window_size_;
            this->window_mean_ = other.window_mean_;
            this->window_std_ = other.window_std_;
            this->is_signal_delayed_ = other.is_signal_delayed_;
            this->max_delay_ = other.max_delay_;

            for (auto i = other.data_.begin(); i!=other.data_.end(); ++i)
                std::cout << " i " <<std::endl;
            //TODO
            this->check = std::bind(&DataContainer::statistics_check,this);

        }
        virtual ~DataContainer();
        bool default_check();
        bool statistics_check();
        std::function<bool(void)> check;
        void updateTime();
        std::string getId();
        void updateData(double new_data);
        std::mutex mtx_;
        void reset();
    
    private:
        ros::Time last_time_;
        int window_size_;
        double window_mean_;
        double window_std_;
        double last_window_std_;
        double max_delay_; 
        std::list<double> data_;
        std::string data_id_;
        bool is_signal_delayed_;
    
    };
};


#endif /* DATA_CONTAINER_H */

