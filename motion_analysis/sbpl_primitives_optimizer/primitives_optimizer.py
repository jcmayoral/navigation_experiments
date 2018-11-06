import yaml
import os
from functools import reduce
import itertools
import pandas as pd
import numpy as np
import time
from scipy.optimize import least_squares
from termcolor import colored

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
min_score = np.min(np.asarray(calculate(results_distance, results_turning, results_time)))
all_scores = np.asarray(calculate(results_distance, results_turning, results_time))

print colored("Best primitives file name: " , "green"), results_config[arg_min]

str_ = results_config[arg_min].replace("_", " ").split()[1:4]
print colored("Looking for ", "magenta", attrs=['blink', 'reverse']), str_

all_params = np.zeros((28, 16))
counter = 0

for k in current_configurations.keys():
    for i in range(len(current_configurations[k])):
        all_params[counter][i] = current_configurations[k][i]
    counter += 1
    if all(x in k for x in str_ ):
        print colored(("Str found at ", k), "red", attrs=['dark'])
        best_params = current_configurations[k]

print colored(("Best parameters set ", best_params),"cyan")

#print len(all_params), len(all_scores)
#all_params_arr = np.resize(all_params, (28,16))
#all_scores_arr = np.resize(np.asarray(all_scores), (28,1))
coefficients = np.linalg.lstsq(all_params, all_scores, rcond=10)[0]
print colored("lstsq coefficients ", "blue"), coefficients
#print "ALL params ", all_params


f = lambda x: np.fabs(np.sum(coefficients*x))
x = best_params
best_1 = least_squares(f, x, method='dogbox', ftol=min_score).x# bounds=bnd).x
print colored('Optimizer 1:', "red"), colored( best_1, 'green')
