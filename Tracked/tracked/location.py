'''
Created on 01.06.2021

@author: Enzo Brancaccio
'''
class Location(object):
    '''
    Persevering a message's location data.
    '''


    def __init__(self, sourceNode, locationId):
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
    
    def isValid(self, sourceNode):
        isSourceNode = not (sourceNode == "")
        return isSourceNode
        