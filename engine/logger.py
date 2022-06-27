from helpers.better_namespace import BetterNamespace


class Logger(BetterNamespace):
    def __init__(self):
        pass
    
    # can only be numbers/strings/lists
    def log(self, name, val):
        self.update({name: val})
        
    def return_log(self):
        return_dict = {}
        for data_name, data in self.items():
            if hasattr(data, '__len__') and len(data) > 1:
                for i in range(len(data)):
                    return_dict[data_name + "_" + str(i)] = data[i]
            else:
                return_dict[data_name] = data
        return return_dict