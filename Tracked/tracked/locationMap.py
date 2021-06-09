'''
Created on 01.06.2021

@author: Enzo Brancaccio
'''

# LocationMap helper class

from .location import Location
 
def addLocations(lmA, lmB, prefix):
    for key, value in lmB.items():
        if(len(prefix) == 0):
            lmA[key] = value
        else:
            if(key == "."):
                lmA[prefix] = value
            else:
                lmA[prefix + "/" + key] = value
    return lmA
    
def removeLocations(lmA, prefix):
    lmCopy = dict()
    for key, value in lmA.items():
        keyStartsWithPrefix = key.rfind(prefix, 0)
        if(not keyStartsWithPrefix == 0):
            lmCopy[key] = value
    return lmCopy
    
def locationSlice(lmA, prefix):
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
    