import random
import yaml
import copy
import rospy

class ResultSaver:
    def __init__(self, output_file_path = 'results/results.txt'):
        self.configuration_queue = list()
        self.results_queue = list()
        self.file_name = output_file_path

    def save_results(self, configuration, results):
        fout = open(self.file_name, "a")

        fout.write("Configuration Params \n" )
        for k, v in configuration.items():
            fout.write(str(k) + ':'+ str(v) + '\n')
        fout.write("Configuration Results \n")
        for k, v in results.items():
            fout.write(str(k) + ':'+ str(v) + '\n')
        fout.write("\n")
        fout.close()

class ConfigurationManager:
    def __init__(self, config_name = 'cfg/params.yaml'):
        file_stream = file(config_name, 'r')
        params = yaml.load(file_stream)
        self.config_params = dict()

        for i in params:
            self.config_params[i] = Param(**params[i])

        self.default_config = copy.deepcopy(self.config_params)
        rospy.loginfo("Configuration Manager is ready")

    def restart_params(self):
        self.config_params = copy.deepcopy(self.default_config)

    def get_new_param_value(self, key):
        return self.config_params[key].get_random_value()

    def get_new_param_values(self, new_param_dict):
        for i in self.config_params:
            new_param_dict[i] = self.get_new_param_value(i)


class Param:
    def __init__(self, min_value, max_value, increment=0.1, default_value = None):
        self.min_value = min_value
        self.max_value = max_value
        self.increment = increment

        #TODO Get Default values from param server
        if default_value is None:
            self.last_value = (max_value - min_value)/2 #Initializing some point in the middle
        else:
            self.last_value = default_value

    def get_random_value(self):
        random_flag = random.randint(0,2)

        if random_flag == 0: #NO MODIFICATIONS
            return self.last_value
        elif random_flag == 1: #INCREMENT
            self.last_value = min(self.increment + self.last_value, self.max_value)
            return self.last_value
        else: #DECREASE VALUE
            self.last_value = max(self.last_value - self.increment, self.min_value)
            return self.last_value
