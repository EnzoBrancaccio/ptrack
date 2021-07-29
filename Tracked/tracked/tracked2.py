'''
Created on 18.06.2021

@author: Enzo Brancaccio
'''

from .location2 import Location2
from rosslt_msgs.msg import LocationHeader

# trying out only one constructor with 1 totally generic parameter
class Tracked2(object):
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
            self.location = Location2()
        else:
            self.location = location
        
        self.location_map = dict([(".", self.location)])
        
    @classmethod
    def withLocationMap(cls, value, location_map):
        newTracked = cls.__new__(cls)
        super(Tracked2, newTracked).__init__()
        cls.value = value
        cls.location_map = location_map
        return newTracked
    
    @classmethod
    def withRossltMsg(cls, value, rosslt_msg):
        newTracked = cls.__new__(cls)
        super(Tracked2, newTracked).__init__()
        cls.value = value
        if(isinstance(rosslt_msg, LocationHeader)):
            for i in range(len(rosslt_msg.paths)):
                cls.location_map[rosslt_msg.paths[i]] = rosslt_msg.locations[i] 
        return newTracked
        