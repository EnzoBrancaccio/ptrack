'''
Created on 06.08.2021

@author: Enzo Brancaccio
'''

import rclpy
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
        # {scource_location: int}
        self.source_locations = dict()
        # {int (location_id): string}, replaces struct locationFunc in C++
        self.locationFunction = dict()
        # list of locationFunctions
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
            if(self.id < 0 or self.id >= len(self.locations)):
                rclpy.logging.LoggingSeverity.WARN(self.node.get_logger(), f"source change to unknown location id {self.id}")
            else:
                self.locations[self.id].set(self.id, msg.new_value)
                
    def create_location(self, location_func, source_location):
        pass
    
    def change_location(self, source_node, location_id, new_value):
        self.msg = SourceChange
        self.msg.source_node(source_node)
        self.msg.location_id(location_id)
        self.msg.new_value(new_value)
        
        if(source_node == self.node.get_name()):
            self.on_source_change(self.msg)
        else:
            self.sc_pub.publish(self.msg)
            
    def current_value(self, location_id):
        locFun = self.locations[location_id]
        return locFun[location_id]