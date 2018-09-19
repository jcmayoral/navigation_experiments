import yaml
import os

from scipy.optimize import least_squares, minimize
import numpy as np
from utilities.configuration import ExperimentSample

file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results/results_sim.txt")
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
        if i.get_result_data(key)["lenght"] > 1:
            res_list = list()
            for z in range(i.get_result_data(key)["lenght"]):
                res_list.append(i.get_result_data(key)["data"][z])
            res_dict[key] = res_list
        else:
            res_dict[key] = i.get_result_data(key)["data"]

    data[str(c)] = [cfg_dict, res_dict]
    c+=1


min_score = 10000000000
best_results = dict()
results_list = list()
coefficients_list = list()

for key, value in data.iteritems():
    overall_score = 0

    #TODO REMOVE TRY EXCEPT
    for k, results_values in value[1].iteritems():
        try:
            for v in results_values:
                overall_score+= np.fabs(v)
        except:
            overall_score+= np.fabs(results_values)

    coefficients = list()
    for k, cfg_values in value[0].iteritems():
        coefficients.append(cfg_values)
    coefficients_list.append(coefficients)

    results_list.append(overall_score)
    if overall_score < min_score:
        min_score = overall_score
        best_results = value[0]

print "MIN Score", min_score
print "MIN BEST CONFIG ", best_results

coefficients_arr = np.asarray(coefficients_list)
best_coefficients = np.linalg.lstsq(coefficients_arr, results_list)[0]
x0 = [i for i in best_results.values()]
print "Proof ", np.sum(best_coefficients * x0)

f = lambda x: np.fabs(np.sum(best_coefficients*x))
x = [15,15,15,45,2.0,0.1]
bnd = ([10.,10.,10., 40., 1.8, 0.05], [30,30,30, 60, 4, 0.15])
best_1 = least_squares(f, x, method='dogbox', bounds=bnd, ftol=min_score).x
print "Optimizer 1: ", best_1
print "Result_1 ", np.sum(best_coefficients*best_1)
x = np.zeros(6)
best_2 = minimize(f,x, bounds=[[10,30],[10,30],[10,30],[40,60],[1.8,2.5],[0.05,0.15]]).x
print "Optimizer 2: ", best_2
print "Result_2 ", np.sum(best_coefficients*best_2)
