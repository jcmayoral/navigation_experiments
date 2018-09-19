import yaml
import os

import numpy as np
from utilities.configuration import ExperimentSample

file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results/results_2.txt")
data = yaml.load(file)

with open(file, 'r') as stream:
    try:
        data = yaml.load(stream)
    except yaml.YAMLError as exc:
        print(exc)

experiment_data = list()
label = list()

for sample in data.iteritems():
    experiment_data.append(ExperimentSample(**sample[1]))

c = 0
data = dict()

for i in experiment_data:
    cfg_dict = dict()
    res_dict = dict()

    for key in i.get_config_keys():
        cfg_dict[key] = i.get_config_data(key)

    for key in i.get_result_keys():
        res_list = list()
        for z in range(i.get_result_data(key)["lenght"]):
            res_list.append(i.get_result_data(key)["data"][z])
        res_dict[key] = res_list

    data[str(c)] = [cfg_dict, res_dict]
    c+=1


min_score = 10000000000
best_results = dict()

for key, value in data.iteritems():
    overall_score = 0
    for k, results_values in value[1].iteritems():
        for v in results_values:
            overall_score+= v
    if overall_score < min_score:
        min_score = overall_score
        best_results = value[0]

print "MIN Score", min_score
print "BEST CONFIG ", best_results
