import yaml
import os

from string import ascii_letters
import numpy as np
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
from utilities.configuration import ExperimentSample

sns.set(style="white")

def analyze_data(environment="fake"):
    file = os.path.join(os.getcwd(), "results/results_"+ environment+".txt")

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
        complete_list = list()

        for key in i.get_config_keys():
            complete_list.append(i.get_config_data(key))

        for key in i.get_result_keys():
            if i.get_result_data(key)["lenght"] > 1:
                for z in range(i.get_result_data(key)["lenght"]):
                    complete_list.append(i.get_result_data(key)["data"][z])
            else:
                complete_list.append(i.get_result_data(key)["data"])
        data.append(complete_list)

    for i in experiment_data[0].get_config_keys():
        label.append(i)

    i = experiment_data[0]

    for j in i.get_result_keys():
        size = i.get_result_data(j)["lenght"]
        for z in range (size):
            label.append(j+str(z))

    d = pd.DataFrame(data=data,
                     columns=label)

    # Compute the correlation matrix
    corr = d.corr()

    # Generate a mask for the upper triangle
    mask = np.zeros_like(corr, dtype=np.bool)
    mask[np.triu_indices_from(mask)] = True

    # Set up the matplotlib figure
    f, ax = plt.subplots(figsize=(45, 45))

    # Generate a custom diverging colormap
    cmap = sns.diverging_palette(100, 130, as_cmap=True)

    # Draw the heatmap with the mask and correct aspect ratio
    sns.heatmap(corr, mask=mask, cmap=cmap, vmax=3, center=0,
                square=True, linewidths=.5, cbar_kws={"shrink": 1})

    plt.show()
