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
    
    def __init__(self, value, y = None):
        '''
        Constructor
        '''
        self.value = value
        self.location = Location()
        self.location_map = dict([(".", self.location)])
        self.rosslt_msg = None
        
        if(not (y is None)):
            if(isinstance(y, Location)):
                self.location = y
                self.location_map = dict([(".", self.location)])
            if(isinstance(y, dict)):
                self.locationMap = y
            if(isinstance(y, LocationHeader)):
                for i in range(len(y.paths)):
                    self.location_map[y.paths[i]] = y.locations[i]
    
    # overloading the dot
    '''
    def __getattr__(self, attr):
        try:
            # not one of object's writable attributes
            if(attr not in self.__dict__):
                pass
            return self.__dict__[attr]
        except KeyError:
            raise AttributeError(attr)
    '''
   
    # overloading the dot . (attribute access)
    # called when attr is not one of object's writable attributes
    def __getattr__(self, attr):
        return th.interpret_dot_attr(self, attr)
    
    # overloading + operator
    # x + y => x.__add__(y)
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
    # x + y => y.__radd__(x)
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
                # no use of 'append' because of setitem overloading
                if(isinstance(inputArg, Tracked)):
                    #self.value.append(inputArg)
                    self[len(self.value)] = inputArg
                    self.location_map = lm.addLocations(self.location_map, inputArg.location_map, str(self.size() - 1))
                else:
                    self[len(self.value)] = th.make_tracked(inputArg)
                    
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
    def __getitem__(self, position):
        isTracked = isinstance(self, Tracked)
        isTrackedList = isinstance(self.value, list)
        if(isTracked and isTrackedList):
            listLength = len(self.value)
            if(position >= listLength):
                raise IndexError(f"Index is {position} and list length is {listLength}")
            trackedValue = self.value[position]
            # negative index: -1 for last list element etc.
            if(position < 0):
                actualPosition = list(self.location_map.keys())[position]
                if(self.location_map[actualPosition].isValid()):    
                    trackedLocation = self.location_map[actualPosition]
                    return th.make_tracked(trackedValue, trackedLocation)
                else:
                    return th.make_tracked(trackedValue)
            else:
                if(self.location_map[str(position)].isValid()):    
                    trackedLocation = self.location_map[str(position)]
                    return th.make_tracked(trackedValue, trackedLocation)
                else:
                    return th.make_tracked(trackedValue)
        else:
            try:
                return self[position]
            except IndexError:
                raise IndexError(f"Index is {position} and list length is {len(self)}")
    
    # overload [] to save update to list in references dictionary       
    def __setitem__(self, position, item):
        isTracked = isinstance(self, Tracked)
        isTrackedList = isinstance(self.value, list)
        if(isTracked and isTrackedList):
            isItemTracked = isinstance(item, Tracked)
            if(isItemTracked):
                if(len(self.value) < position):
                    self.value[position] = item.value
                else:
                    self.value.append(item.value)
                self.location_map = lm.addLocations(self.location_map, item.location_map, str(position))
            else:
                if(len(self.value) < position):
                    self.value[position] = item
                else:
                    self.value.append(item)
                self.location_map = lm.addLocations(self.location_map, dict(), "")