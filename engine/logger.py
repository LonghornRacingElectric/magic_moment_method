from helpers.better_namespace import BetterNamespace


class Logger(BetterNamespace):
    """
    Logs dependent states & values calculated in solver, saved for post-analysis
    """
    def __init__(self):
        pass
    

    def log(self, name:str, val:object):
        """
        Logs input value with given name

        Args:
            name (str): key
            val (object): numbers, strings, iterables
        """
        self.update({name: val})
        

    def return_log(self):
        """
        Takes logged values and parses them into dictionary.

        Iterables will be broken up like the following:

        self[vals] -> list([1, 2, 3])
        self[vals_1] -> 1
        self[vals_2] -> 2
        self[vals_3] -> 3

        Returns:
            dict: returns dependent states & values in dictionary
        """
        return_dict = {}
        for data_name, data in self.items():
            if hasattr(data, '__len__') and len(data) > 1:
                for i in range(len(data)):
                    return_dict[data_name + "_" + str(i)] = data[i]
            else:
                return_dict[data_name] = data
        return return_dict