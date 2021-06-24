'''
Created on 24.06.2021

@author: Enzo Brancaccio
'''

from .location import Location
from .tracked import Tracked

def makeTracked(value, location = None):
    if(isinstance(value, Tracked)):
        return value
    else:
        if(location is None):
            return Tracked(value, Location())
        else:
            return Tracked(value, location)

