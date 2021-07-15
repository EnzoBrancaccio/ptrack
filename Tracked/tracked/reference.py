'''
Created on 27.06.2021

@author: Enzo Brancaccio
'''

from .tracked import Tracked

class Reference(object):
    '''
    classdocs
    '''


    def __init__(self, trackedList, position):
        '''
        Constructor
        '''
        self.trackedList = trackedList
        self.position = position
        
    # trying to overload =
    def __setattr__(self, attrName, other):
        isTrackedList = isinstance(self.trackedList, list)
        isTrackedOther = isinstance(other, Tracked)
        if(isTrackedList and isTrackedOther):
            self.trackedList[self.position] = other.value