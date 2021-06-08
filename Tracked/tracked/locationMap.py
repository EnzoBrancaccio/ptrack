'''
Created on 01.06.2021

@author: Enzo Brancaccio
'''

from .location import Location

class LocationMap(object):
    '''
    classdocs
    '''


    def __init__(self):
        '''
        Constructor
        '''
        self.locationMap = dict()
    
    def addLocations(self, lmA, lmB, prefix):
        for key, value in lmB.items():
            if(len(prefix) == 0):
                lmA[key] = value
            else:
                if(key == "."):
                    lmA[prefix] = value
                else:
                    lmA[prefix + "/" + key] = value
        return lmA
    
    def removeLocations(self, lmA, prefix):
        lmCopy = dict()
        for key, value in lmA.items():
            keyStartsWithPrefix = key.rfind(prefix, 0)
            if(not keyStartsWithPrefix == 0):
                lmCopy[key] = value
        return lmCopy
    
    def locationSlice(self, lmA, prefix):
        lmCopy = dict()
        for key, value in lmA.items():
            keyStartsWithPrefix = key.rfind(prefix, 0)
            if(keyStartsWithPrefix == 0):
                newKey = key[len(prefix):]
                if(newKey == ""):
                    lmCopy["."] = value
                else:
                    lmCopy[newKey[1:]] = value
        return lmCopy
    