"""_summary_
Creating a class that follows this
https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_spatial_force.html
"""
class SpacialForce:
    def __init__(self, force, moment):
        self._force = list(force)
        self._moment = list(moment)

    def __iter__(self):
        for comp in self.data:
            yield comp
    
    def __add__(self, other):
        forces = [self._force[i] + other._force[i] for i in len(self._force)]
        moments = [self._moment[i] + other._moment[i] for i in len(self._moment)]
        return SpacialForce(forces, moments)

    def __sub__(self, other):
        forces = [self._force[i] - other._force[i] for i in len(self._force)]
        moments = [self._moment[i] - other._moment[i] for i in len(self._moment)]
        return SpacialForce(forces, moments)

    @property
    def data(self):
        return [*self._force, *self._moment]