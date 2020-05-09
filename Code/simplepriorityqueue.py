import numpy as np

class SimplePriorityQueue:
    def __init__(self):
        self.key_list = np.array([], dtype=object)
        self.value_list = np.array([], dtype=object)
    
    def put(self, key, value = None):
        self.key_list = np.append(self.key_list, key)
        self.value_list = np.append(self.value_list, value)
        
        ind = self.key_list.argsort(kind ='heapsort')
        self.key_list = self.key_list[ind]
        self.value_list = self.value_list[ind]
        
    def get(self):
        if len(self.key_list) != 0:
            key = self.key_list[0]
            value = self.value_list[0]
            self.key_list = np.delete(self.key_list, [0])
            self.value_list = np.delete(self.value_list, [0])
            return key, value
        else:
            return None, None
        
    def isempty(self):
        return len(self.key_list) == 0
