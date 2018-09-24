#!/usr/bin/env python
import sys
import os
import yaml
import rospy

from utilities.data_analysis import analyze_data, get_best_configuration

__author__ = 'banos'

if __name__ == '__main__':
    analyze_data(environment="corridor_toru15_real_10m")
    get_best_configuration(environment="corridor_toru15_real_10m")
