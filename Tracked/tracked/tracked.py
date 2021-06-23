'''
Created on 01.06.2021

@author: Enzo Brancaccio
'''

from .location import Location
from rosslt_msgs.msg import LocationHeader
import copy
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