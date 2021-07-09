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
        self.references = dict()
            
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
    
    # overloading the dot .
    def __getattr__(self, attr):
        try:
            # not one of object's writable attributes
            if(attr not in self.__dict__):
                pass
            return self.__dict__[attr]
        except KeyError:
            raise AttributeError(attr)
    
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
                    
    def front(self):
        if(isinstance(self, Tracked)):
            if(isinstance(self.value, list)):
                return self.value[0]
            
    def back(self):
        if(isinstance(self, Tracked)):
            if(isinstance(self.value, list)):
                return self.value[-1]
    
    # overload [] for Tracked with value list
    '''
    def __getitem__(self, position):
        if(isinstance(self, Tracked)):
            if(isinstance(self.value, list)):
                listLength = len(self.value)
                if(position > listLength):
                    raise IndexError(f"Index is {position} and list length is {listLength}")
                return self.value[position]
    '''
           
    # overload assignment to save LHS and RHS values
    def __setattr__(self, name, val):
        # first, else attribute may not exist (e. g. Tracked.value)
        super().__setattr__(name, val)
        if(isinstance(self, Tracked)):
            if(isinstance(self.value, list)):
                if(name in self.__dict__):
                    print(self.value)
                    #if(getattr(self, "references", False)):
                    #    setattr(self, "references", dict())
                    # value appended to list so index will be length of list prior to insertion
                    newIndex = len(self.value)
                    # save key-value pair (index, value) in references dict
                    #self.references[newIndex] = val
                else:
                    raise AttributeError(f"Unknown attribute {name}")