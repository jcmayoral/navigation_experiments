/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   CPUMonitor.h
 * Author: banos
 *
 * Created on October 10, 2018, 10:34 AM
 */
#include<vector>
#include<numeric>
#include<fstream>
#include <ros/ros.h>

#ifndef CPUMONITOR_H
#define CPUMONITOR_H

class CPUMonitor {
public:
    CPUMonitor();
    CPUMonitor(const CPUMonitor& orig);
    virtual ~CPUMonitor();
    double getUsage();
    void updateData(int index, double value);

protected:
    double getActiveTime();

private:
    double last_cpu_usage_;
    double last_cpu_total_;
    std::vector <double> cpu_usage_;
    
};

#endif /* CPUMONITOR_H */

