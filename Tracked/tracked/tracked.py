'''
Created on 01.06.2021

@author: Enzo Brancaccio
'''

from .location import Location

class Tracked(object):
    '''
    classdocs
    '''

    def __init__(self, value):
        '''
         Constructor
        '''
        self.value = value
        self.location["."] = Location()