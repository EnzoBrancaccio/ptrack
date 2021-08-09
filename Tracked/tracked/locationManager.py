'''
Created on 06.08.2021

@author: Enzo Brancaccio
'''

from rclpy.node import Node
from rosslt_msgs.msg import SourceChange
from rosslt_msgs.srv import GetValue
from .location import Location

class LocationManager(object):
    '''
    classdocs
    '''


    def __init__(self, node):
        '''
        Constructor
        '''
        self.source_locations = dict()
        self.locations = list()
        self.get_value_service = None
        self.node = Node
        
        self.sc_sub = node.create_subscription(SourceChange, "/sc", self.on_source_change(self), 10)
        self.sc_pub = node.create_publisher(SourceChange, "/sc", 10)
        self.get_value_service = node.create_service(GetValue, node.get_name() + "/get_slt_value", self.on_get_value(self, self), 10)
        
    def on_get_value(self, request, response):
        if(len(self.locations) > request.location_id >= 0):
            response.set_current_value(self.locations[request.location_id].get(request.location_id))
            response.set_valid_id(True)
        else:
            response.set_valid_id(False)
    
    def on_source_change(self, msg):
        if(msg.source_node == self.node.get_name()):
            self.id = msg.location_id
            if(0 > self.id >= len(self.locations)):
                #RCLCPP_WARN(node.get_logger(), "source change to unknown location id %d", id);
                pass
            else:
                self.locations[self.id].set(self.id, msg.new_value)