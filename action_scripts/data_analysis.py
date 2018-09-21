#!/usr/bin/env python
import sys
import os
import yaml
import rospy

from utilities.data_analysis import analyze_data, get_best_configuration

__author__ = 'banos'

if __name__ == '__main__':
    analyze_data()
    get_best_configuration()
