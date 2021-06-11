'''
Created on 01.06.2021

@author: Enzo Brancaccio
'''

from .location import Location

class Tracked(object):
    '''
    classdocs
    '''

    def __init__(self, value, locationMap = None, location = None):
        '''
         Constructor
        '''
        self.value = value
        
        if locationMap is None:
            self.locationMap = dict([(".", Location())])
        else:
            self.locationMap = locationMap
            
        if location is None:
            self.location = Location()
        else:
            self.location = location