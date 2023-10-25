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
        original_list = d.replace("[", "").replace("]", "").replace("array(","").replace(")","").split(r", ")
        original_list = [item.replace('\n', '').split(',') for item in original_list]
        cleaned_list = []
        for sublist in original_list:
            sub_cleaned = [x.strip().split(',') for x in sublist if x.strip()]
            cleaned_list.extend(sub_cleaned)
        return np.array(cleaned_list).astype(float)
    
    @staticmethod
    def speical_trans2(d):
        if not d :
            return []
        if d =="[]":
            return []
        return        np.array(d.replace("[", "").replace("]", "").replace("array(","").replace(")","").split(',')).astype(float)

    
