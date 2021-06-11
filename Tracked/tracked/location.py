'''
Created on 01.06.2021

@author: Enzo Brancaccio
'''
class Location(object):
    '''
    Persevering a message's location data.
    '''


    def __init__(self, sourceNode = "", locationId = 0):
        '''
        Constructor
        '''
        self.sourceNode = sourceNode
        self.locationId = locationId
        self.expression = ""
      
    # overloading boolean operator  
    def __eq__(self, other):
        sameSource = (self.sourceNode == other.sourceNode)
        sameId = (self.locationId == other.locationId)
        return (sameSource and sameId)
    
    def isValid(self):
        isSourceNode = not (self.sourceNode == "")
        return isSourceNode
        