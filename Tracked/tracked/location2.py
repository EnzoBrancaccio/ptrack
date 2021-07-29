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


    def __init__(self, source_node = "", location_id = 0):
        '''
        Constructor
        '''
        self.source_node = source_node
        self.location_id = location_id
        self.expression = ""
    
    # constructor for rosslt location message as argument    
    @classmethod
    def withRossltMsg(cls, rossltMsg = None):
        newLocation = cls.__new__(cls)
        super(Location2, newLocation).__init__()
        if not (rossltMsg is None):
            cls.source_node = rossltMsg.source_node
            cls.location_id = rossltMsg.location_id
        return newLocation
    
    
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