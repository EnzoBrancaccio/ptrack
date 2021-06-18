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


    def __init__(self, value, y = None):
        '''
        Constructor
        '''
        self.value = value
        self.location = Location2()
        self.location_map = dict([(".", self.location)])
        self.rosslt_msg = None
        
        if(not (y is None)):
            if(isinstance(y, Location2)):
                self.location = y
                self.location_map = dict([(".", self.location)])
            if(isinstance(y, dict)):
                self.locationMap = y
            if(isinstance(y, LocationHeader)):
                for i in range(len(y.paths)):
                    self.location_map[y.paths[i]] = y.locations[i]
        