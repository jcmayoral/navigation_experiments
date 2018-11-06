import yaml
import os
from functools import reduce
import itertools
import pandas as pd
import numpy as np

config_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "current_configurations.yaml")
file_stream = file(config_file, 'r')
print os.listdir('.')
current_configurations = yaml.load(file_stream)
if config_file == current_configurations:
    print "FILE NOT FOUND"

config_flatten_list = list()
for key, value in current_configurations.iteritems():
    current_configurations[key] = list(itertools.chain(*(i if isinstance(i, list) else (i,) for i in value)))

#for key, value in current_configurations.iteritems():
#    print key, value


results = pd.ExcelFile('results.xlsx')
sh1 = results.parse('Sheet')


results = list()
results_config = sh1["CONFIGURATION"]
results_distance = sh1['DISTANCE']
results_turning = sh1['TURNING']
results_time = sh1["TIME"]

calculate = lambda x,y,z: x+y+z
arg_min = np.argmin(np.asarray(calculate(results_distance, results_turning, results_time)))

print results_config[arg_min]
