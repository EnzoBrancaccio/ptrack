'''
Created on 01.06.2021

@author: Enzo Brancaccio
'''

import copy
import math
import tracked.locationMap as lm
import tracked.trackingHelpers as th

from .location import Location
from rosslt_msgs.msg import LocationHeader
from numbers import Number

class Tracked(object):
    '''
    classdocs
    '''

    def __init__(self, value, location = None):
        '''
         Constructor
        '''
        self.value = value
            
        if location is None:
            self.location = Location()
        else:
            self.location = location
        
        self.location_map = dict([(".", self.location)])
        
    @classmethod
    def withLocationMap(cls, value, location_map):
        newTracked = cls.__new__(cls)
        super(Tracked, newTracked).__init__()
        cls.value = value
        cls.location_map = location_map
        return newTracked
    
    @classmethod
    def withRossltMsg(cls, value, rosslt_msg):
        newTracked = cls.__new__(cls)
        super(Tracked, newTracked).__init__()
        cls.value = value
        if(isinstance(rosslt_msg, LocationHeader)):
            for i in range(len(rosslt_msg.paths)):
                cls.location_map[rosslt_msg.paths[i]] = rosslt_msg.locations[i] 
        return newTracked
    
    # overloading + operator 
    def __add__(self, other):
        isSelfTracked = isinstance(self, Tracked)
        isOtherTracked = isinstance(other, Tracked)
        if(isSelfTracked and (not isOtherTracked)):
            copiedVar = copy.deepcopy(self)
            copiedVar.value = self.value + other
            expr = copiedVar.location_map["."].expression
            copiedVar.location_map["."].expression = expr + str(other) + ";+;"
            return copiedVar
        elif(isSelfTracked and isOtherTracked):
            copiedVar = copy.deepcopy(self)
            copiedVar.value = self.value + other.value
            expr = copiedVar.location_map["."].expression
            copiedVar.location_map["."].expression = expr + str(other.value) + ";+;"
            return copiedVar
        else:
            newValue = self + other
            return newValue
        
    # if left hand type is not Tracked   
    def __radd__(self, other):
        isSelfTracked = isinstance(self, Tracked)
        if(isSelfTracked):
            copiedVar = copy.deepcopy(self)
            copiedVar.value = other + self.value # str concatenation
            expr = copiedVar.location_map["."].expression
            copiedVar.location_map["."].expression = expr + str(other) + ";swap;+;"
            return copiedVar
        
    # overloading - operator 
    def __sub__(self, other):
        isSelfTracked = isinstance(self, Tracked)
        isOtherTracked = isinstance(other, Tracked)
        if(isSelfTracked and (not isOtherTracked)):
            copiedVar = copy.deepcopy(self)
            copiedVar.value = self.value - other
            expr = copiedVar.location_map["."].expression
            copiedVar.location_map["."].expression = expr + str(other) + ";-;"
            return copiedVar
        elif(isSelfTracked and isOtherTracked):
            copiedVar = copy.deepcopy(self)
            copiedVar.value = self.value - other.value
            expr = copiedVar.location_map["."].expression
            copiedVar.location_map["."].expression = expr + str(other.value) + ";-;"
            return copiedVar
        else:
            newValue = self - other
            return newValue
        
    # if left hand type is not Tracked   
    def __rsub__(self, other):
        isSelfTracked = isinstance(self, Tracked)
        if(isSelfTracked):
            copiedVar = copy.deepcopy(self)
            copiedVar.value = other - self.value
            expr = copiedVar.location_map["."].expression
            copiedVar.location_map["."].expression = expr + str(other) + ";swap;-;"
            return copiedVar
    
    # overloading * operator    
    def __mul__(self, other):
        isSelfTracked = isinstance(self, Tracked)
        isOtherTracked = isinstance(other, Tracked)
        if(isSelfTracked and isOtherTracked):
            copiedVar = copy.deepcopy(self)
            copiedVar.value = self.value * other.value
            if(self.location_map["."].isValid()):
                isOtherNull = other.value == 0
                isOtherNumber = isinstance(other.value, Number)
                if(isOtherNull and isOtherNumber):
                    copiedVar.location_map["."] = other.location
                    expr = copiedVar.location_map["."].expression
                    copiedVar.location_map["."].expression = expr + str(self) + ";swap;*;"
                    return copiedVar
                copiedVar.location_map["."] = self.location
                expr = copiedVar.location_map["."].expression
                copiedVar.location_map["."].expression = expr + str(other.value) + ";*;"
                return copiedVar
        elif(isSelfTracked):
            copiedVar = copy.deepcopy(self)
            copiedVar.value = self.value * other
            expr = copiedVar.location_map["."].expression
            copiedVar.location_map["."].expression = expr + str(other) + ";*;"
            isOtherNull = other == 0
            isOtherNumber = isinstance(other, Number)
            if(isOtherNull and isOtherNumber):
                copiedVar.location_map["."] = Location()
            return copiedVar
        else:
            newValue = self * other
            return newValue
        
    # if left hand type is not Tracked   
    def __rmul__(self, other):
        isSelfTracked = isinstance(self, Tracked)
        if(isSelfTracked):
            copiedVar = copy.deepcopy(self)
            copiedVar.value = other * self.value
            expr = copiedVar.location_map["."].expression
            copiedVar.location_map["."].expression = expr + str(other) + ";swap;*;"
            isOtherNull = other == 0
            isOtherNumber = isinstance(other, Number)
            if(isOtherNumber and isOtherNull):
                copiedVar.location_map["."] = Location()
            return copiedVar
        
    # overloading / operator 
    def __truediv__(self, other):
        isSelfTracked = isinstance(self, Tracked)
        isOtherTracked = isinstance(other, Tracked)
        if(isSelfTracked and isOtherTracked):
            copiedVar = copy.deepcopy(self)
            copiedVar.value = self.value / other.value
            expr = copiedVar.location_map["."].expression
            copiedVar.location_map["."].expression = expr + str(other.value) + ";/;"
            return copiedVar
        elif(isSelfTracked and (not isOtherTracked)):
            copiedVar = copy.deepcopy(self)
            copiedVar.value = self.value / other
            expr = copiedVar.location_map["."].expression
            copiedVar.location_map["."].expression = expr + str(other) + ";/;"
            return copiedVar
        else:
            newValue = self / other
            return newValue
        
    # if left hand type is not Tracked   
    def __rtruediv__(self, other):
        isSelfTracked = isinstance(self, Tracked)
        if(isSelfTracked):
            copiedVar = copy.deepcopy(self)
            copiedVar.value = other / self.value
            expr = copiedVar.location_map["."].expression
            copiedVar.location_map["."].expression = expr + str(other) + ";swap;/;"
            return copiedVar
        
    def sin(self):
        isSelfTracked = isinstance(self, Tracked)
        isValueNumber = isinstance(self.value, Number)
        if(isSelfTracked and isValueNumber):
            copiedVar = copy.deepcopy(self)
            copiedVar.value = math.sin(self.value)
            expr = copiedVar.location_map["."].expression
            copiedVar.location_map["."].expression = expr + "sin;"
            return copiedVar
        
    def cos(self):
        isSelfTracked = isinstance(self, Tracked)
        isValueNumber = isinstance(self.value, Number)
        if(isSelfTracked and isValueNumber):
            copiedVar = copy.deepcopy(self)
            copiedVar.value = math.cos(self.value)
            expr = copiedVar.location_map["."].expression
            copiedVar.location_map["."].expression = expr + "cos;"
            return copiedVar
        
    # vector methods: Choosing its Python counterpart "list"
    def size(self):
        if(isinstance(self, Tracked)):
            if(isinstance(self.value, list)):
                return len(self.value)
    
    # everything in list is of / will be turned into type Tracked        
    def push_back(self, inputArg):
        if(isinstance(self, Tracked)):
            if(isinstance(self.value, list)):
                if(isinstance(inputArg, Tracked)):
                    self.value.append(inputArg)
                    self.location_map = lm.addLocations(self.location_map, inputArg.location_map, str(self.size() - 1))
                else:
                    self.value.append(th.makeTracked(inputArg))
                    
    def pop_back(self):
        if(isinstance(self, Tracked)):
            if(isinstance(self.value, list)):
                del(self.value[-1])
                self.location_map = lm.removeLocations(self.location_map, str(self.size()))
                
    def clear(self):
        if(isinstance(self, Tracked)):
            if(isinstance(self.value, list)):
                self.value.clear()
                self.location_map = lm.locationSlice(self.location_map, ".")
                    
                    
        