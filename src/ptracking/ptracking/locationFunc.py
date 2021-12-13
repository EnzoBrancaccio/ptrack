'''
Created on 17.08.2021

@author: Enzo Brancaccio
'''

'''
equivalent to C++ struct LocationFunc
provides an interface with easy access to values
'''
class LocationFunc(object):
    '''
    classdocs
    '''


    def __init__(self, location_id, new_value):
        '''
        Constructor
        '''
        # int
        self.location_id = location_id
        # generic
        self.new_value = new_value
        
    def get_value(self, location_id):
        if(self.location_id == location_id):
            return self.new_value