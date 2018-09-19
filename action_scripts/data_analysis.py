import yaml
import os

from string import ascii_letters
import numpy as np
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

sns.set(style="white")

class ExperimentSample:
    def __init__(self, config, results):
        self.configuration = config
        self.results = results

    def get_config_data(self, key):
        return self.configuration[key]

    def get_config_keys(self):
        return self.configuration.keys()

    def get_result_keys(self):
        return self.results.keys()

    def get_result_data(self, key):
        return self.results[key]

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

data = list()
c = 0

for i in experiment_data:
    cfg_list = list()
    res_list = list()

    complete_list = list()

    for key in i.get_config_keys():
        complete_list.append(i.get_config_data(key))

    for key in i.get_result_keys():
        for z in range(i.get_result_data(key)["lenght"]):
            complete_list.append(i.get_result_data(key)["data"][z])

    print len(complete_list)
    data.append(complete_list)

for i in experiment_data[0].get_config_keys():
    label.append(i)

print label, "BEFORE"
i = experiment_data[0]

for j in i.get_result_keys():
    size = i.get_result_data(j)["lenght"]
    for z in range (size):
        print z , " OF ", size, j
        label.append(j+str(z))

print "LEN LABEL" , len(label)

d = pd.DataFrame(data=data,
                 columns=label)

# Compute the correlation matrix
corr = d.corr()

# Generate a mask for the upper triangle
mask = np.zeros_like(corr, dtype=np.bool)
mask[np.triu_indices_from(mask)] = True

# Set up the matplotlib figure
f, ax = plt.subplots(figsize=(20, 20))

# Generate a custom diverging colormap
cmap = sns.diverging_palette(20, 10, as_cmap=True)

# Draw the heatmap with the mask and correct aspect ratio
sns.heatmap(corr, mask=mask, cmap=cmap, vmax=3, center=0,
            square=True, linewidths=.5, cbar_kws={"shrink": 1})


plt.show()
