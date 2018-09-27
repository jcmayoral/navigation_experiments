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
//#include <boost/thread/thread.hpp>
//#include <boost/thread/mutex.hpp>
//boost::mutex mtx; 
 
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
        DataContainer(std::string id = "data", bool required_statistics=true);
        virtual ~DataContainer();
        bool default_check();
        bool statistics_check();
        std::function<bool(void)> check;
        void updateTime();
        std::string getId();
        void updateData(double new_data);
    
    private:
        ros::Time last_time_;
        int window_size_;
        double window_mean_;
        double window_std_;
        double max_delay_; 
        std::list<double> data_;
        std::string data_id_;
        std::mutex* mtx;
    };
};


#endif /* DATA_CONTAINER_H */

