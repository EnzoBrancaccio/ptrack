'''
Created on 01.06.2021

@author: Enzo Brancaccio
'''

from src.rosslt_msgs.msg import Location as rosLocationMsg

class Location(object):
    '''
    Persevering a message's location data.
    '''
    
    def __init__(self, x = None, y = None):
        '''
        Constructor
        '''
        self.source_node = ""
        self.location_id = 0
        self.expression = ""
        
        if(not (x is None)):
            if(isinstance(x, str)):
                self.source_node = x
            if(isinstance(x, rosLocationMsg)):
                self.source_node = x.source_node
                self.location_id = x.location_id
                self.expression = x.expression
        
        if(not(y is None)):
            if(isinstance(y, int)):
                self.location_id = y           
      
    # overloading boolean operator  
    def __eq__(self, other):
        """Override boolean operator
        
        Keyword arguments:
        self -- Location object
        other -- Other location object
        
        Compares two Location objects based on source_node and location_id
        """
        sameSource = (self.source_node == other.source_node)
        sameId = (self.location_id == other.location_id)
        return (sameSource and sameId)
    
    def isValid(self):
        """Checks if Location is valid
        
        Keyword arguments:
        self -- Location object
        
        Location is valid if source_node is not empty
        """
        isSourceNode = not (self.source_node == "")
        return isSourceNode
    
    # operator rosslt_msgs::msg::Location () const;
    # not sure it's possible in Python (typing)
    def makeRossltLocationMsg(self):
        locationMsg = rosLocationMsg()
        locationMsg.source_node = self.source_node
        locationMsg.location_id = self.location_id
        locationMsg.expression = self.expression
        return locationMsg