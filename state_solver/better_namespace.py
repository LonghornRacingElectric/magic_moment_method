import types

types.SimpleNamespace()

class BetterNamespace(types.SimpleNamespace):
    def __init__(self):
        pass
    
    def values(self):
        return [x for x in self.__dict__.values()]
    
    def keys(self):
        return [x for x in self.__dict__.keys()]
    
    def items(self):
        return [[x,y] for x,y in self.__dict__.items()]
    
    def update(self, dict):
        self.__dict__.update(dict)