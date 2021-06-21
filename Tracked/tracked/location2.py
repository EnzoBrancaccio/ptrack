'''
Created on 18.06.2021

@author: Enzo Brancaccio
'''

from rosslt_msgs.msg import Location as rosLocationMsg

# trying out only one constructor with 2 totally generic parameters
class Location2(object):
    '''
    classdocs
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
        
        if(not(y is None)):
            if(isinstance(y, int)):
                self.location_id = y
    
    
    # overloading boolean operator  
    def __eq__(self, other):
        sameSource = (self.source_node == other.source_node)
        sameId = (self.location_id == other.location_id)
        return (sameSource and sameId)
    
    def isValid(self):
        isSourceNode = not (self.source_node == "")
        return isSourceNode
    
    # operator rosslt_msgs::msg::Location () const;
    # not sure it's possible in Python (typing)
    def makeRossltLocationMsg(self):
        locationMsg = rosLocationMsg() 
        locationMsg.source_node = self.source_node
        locationMsg.location_id = self.location_id
        return locationMsg