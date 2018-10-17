import os
import re

class ROSPlannerGetter:
    def __init__(self):
        plugins = os.popen("rospack plugins --attrib=plugin nav_core").read()
        plugins = plugins.splitlines()

        plugin_name_type = list()

        for i in plugins:
            j = i.split()
            c_file = open(j[1],"r")
            flags = [False,False,False]

            for line in c_file:
                index_name = line.find("name")
                index_type = line.find("type")
                base_class_index = line.find("base_class_type")

                if index_name > 0 and not flags[0]:
                    name = re.findall(r'"(.*?)"', line[index_name:])[0]
                    flags[0] = True
                if index_type>0 and not flags[1]:
                    plugin_type = re.findall(r'"(.*?)"', line[index_type:])[0]
                    flags[1] = True
                if base_class_index > 0 and not flags[2]:
                    plugin_base_type = re.findall(r'"(.*?)"', line[base_class_index:])[0]
                    flags[2] = True

                if all(flags):
                    plugin_name_type.append([name, plugin_type, plugin_base_type])
                    flags = [False,False,False]

        self.available_global_planners = list()
        self.available_local_planners = list()

        for n_t in plugin_name_type:
            print n_t[0], n_t[2]
            if n_t[2] == "nav_core::BaseGlobalPlanner":
                self.available_global_planners.append(n_t[0])

            if n_t[2] == "nav_core::BaseLocalPlanner":
                self.available_local_planners.append(n_t[0])

        for i in range(len(self.available_global_planners)):
            print "Available GP ", i , self.available_global_planners[i]


        for i in range(len(self.available_local_planners)):
            print "Available LP ", i , self.available_local_planners[i]

        print "finish"

    def getGlobalPlanners(self):
        return self.available_global_planners

    def getLocalPlanners(self):
        return self.available_local_planners
