'''
Created on 01.06.2021

@author: Enzo Brancaccio
'''

# LocationMap helper class
# C++ LocationMap = std::unordered_map<std::string, Location>;
# Here: LocationMap is dictionary {"string": location}
 
def addLocations(lmA, lmB, prefix):
    """Adds locations to a location map (type: dictionary)
        
    Keyword arguments:
    lmA -- Location map locations are added to
    lmB -- Location map providing new locations
    prefix -- Key in location map dictionary
        
    Goes through key-value pairs ("string": location) of location map lmB
    If no prefix is given, just all key-value pairs are also added to location map LmA
    If a prefix is given, it's used to create a new key for the entry
    """
    for key, value in lmB.items():
        if(len(prefix) == 0):
            lmA[key] = value
        else:
            if(key == "."):
                lmA[prefix] = value
            else:
                newKey = prefix + "/" + key
                lmA[newKey] = value
    return lmA
    
def removeLocations(lmA, prefix):
    """Removes locations with a given prefix
        
    Keyword arguments:
    lmA -- Location map to delete location from
    prefix -- Key or part of key in location map dictionary
        
    Creates a copy of provided location map lmA
    Copy contains all key-value pairs from lmA in which the key does not start with the given prefix
    """
    lmCopy = dict()
    for key, value in lmA.items():
        keyStartsWithPrefix = key.rfind(prefix, 0)
        if(not keyStartsWithPrefix == 0):
            lmCopy[key] = value
    return lmCopy
    
def locationSlice(lmA, prefix):
    """Recovers locations with a given prefix
        
    Keyword arguments:
    lmA -- Location map to recover location from
    prefix -- Key or part of key in location map dictionary
        
    Creates a copy of provided location map lmA
    Copy contains all key-value pairs from lmA in which the key does start with the given prefix
    """
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
    