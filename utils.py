import numpy as np


class Utils:
    @staticmethod
    def string_to_float_list(d):
        if not d:
            return []
        if d == "[]" :
            return []
        
        return np.array(d.replace("[", "").replace("]", "").split(r", ")).astype(float)
    
    @staticmethod
    def speical_trans(d):
        if not d :
            return []
        if d =="[]":
            return []
        return np.array(d.replace("[", "").replace("]", "").replace("array(","").replace(")","").split(r", ")).astype(float)
    
    @staticmethod
    def speical_trans2(d):
        if not d :
            return []
        if d =="[]":
            return []
        return        np.array(d.replace("[", "").replace("]", "").replace("array(","").replace(")","").split(',')).astype(float)

    