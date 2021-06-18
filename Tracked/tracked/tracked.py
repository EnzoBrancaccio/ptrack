'''
Created on 01.06.2021

@author: Enzo Brancaccio
'''

from .location import Location
from rosslt_msgs.msg import LocationHeader

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
        if(isinstance(self, Tracked)):
            self.copy = self.a
            self.copy.value = self.a.value + other.a.value
            self.expr = self.copy.location_map["."].expression
            self.copy.location_map["."].expression = self.expr + str(other.a.value) + ";+;"
            return self.copy
        elif(isinstance(other, Tracked)):
            self.copy = other.a
            self.copy.value = self.a.value + other.a
            self.expr = self.copy.location_map["."].expression
            self.copy.location_map["."].expression = str(self.a.value) + self.expr + ";swap;+;"
            return self.copy
        else:
            self.b = self.a.value + other.a.value
            return self.b