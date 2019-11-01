# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import numpy as np

class FIR_filter:
    
    def __init__(self,_coefficients):
        self.ntap = len(_coefficients)
        self.buffer = np.zeros(self.ntap)
        self.coefficient = _coefficients
 
        
    def filter(self,v):
        output=0
        
        for i in range(self.ntap - 1):
            self.buffer[self.ntap - i - 1] = self.buffer[self.ntap -i -2]
            
        self.buffer[0] = v
            
        for i in range(self.ntap):
            output += self.buffer[i]*self.coefficient[i]
        
        return output
    
