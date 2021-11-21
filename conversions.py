"""
For use in converting common units and reference frames within the MMM solver framework
Units:
    Length: meters
    Mass: kg
"""
class Conversions():
    @staticmethod
    def inch_to_meter(x):
        return x/39.37

    @staticmethod
    def meter_to_inch(x):
        return x*39.37

    @staticmethod
    def lbf_to_Nm(x):
        return x / (.0254 * .224)
    @staticmethod
    def Nm_to_lbf(x):
        return x * (.0254 * .224)



