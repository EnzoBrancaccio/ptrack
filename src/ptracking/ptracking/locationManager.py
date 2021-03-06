'''
Created on 06.08.2021

@author: Enzo Brancaccio
'''

import rclpy

from rclpy.qos import qos_profile_services_default
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from src.rosslt_msgs.msg import SourceChange
from src.rosslt_msgs.srv import GetValue

class LocationManager(object):
    '''
    classdocs
    '''

    '''
    Regarding source_location:
    https://docs.python.org/3/library/inspect.html#the-interpreter-stack
    returns named tuple, so source_location is simply an instance of named tuple
    in TrackingNode's loc-function a named tuple will be created from inspect.stack
    source_locations is just a list of such named tuples
    '''

    def __init__(self, node):
        '''
        Constructor
        '''
        # {scource_location: int}, note that source_location is a string
        self.source_locations = dict()
        # replaces C++ locations, list of LocationFunc instances
        self.locations = list()
        self.get_value_service = None
        self.node = node
        
        self.sc_sub = self.node.create_subscription(SourceChange, "/sc", self.on_source_change, qos_profile = qos_profile_services_default, callback_group = MutuallyExclusiveCallbackGroup())
        self.sc_pub = self.node.create_publisher(SourceChange, "/sc", qos_profile = qos_profile_services_default, callback_group = MutuallyExclusiveCallbackGroup())
        self.get_value_service = self.node.create_service(GetValue, node.get_name() + "/get_slt_value", self.on_get_value)
        
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
        # source_locations is dict with key: source_location, value: int
        if(source_location not in self.source_locations):
            self.locations.append(location_func)
            self.source_locations[source_location] = len(self.locations) - 1
            return len(self.locations) - 1
        else:
            return self.source_locations[source_location]
    
    def get_location_id(self, source_location):
        # source_locations is dict with key: source_location, value: int
        # check if source_location is a key in self.source_locations
        if(source_location in self.source_locations):
            self.it = self.source_locations[source_location]
            return self.it
        else:
            return -1
    
    def change_location(self, source_node, location_id, new_value):
        self.msg = SourceChange()
        self.msg.source_node = source_node
        self.msg.location_id = location_id
        self.msg.new_value = new_value
        
        if(source_node == self.node.get_name()):
            self.on_source_change(self.msg)
        else:
            self.sc_pub.publish(self.msg)
    
    # self.locations is list of locationFunc instances        
    def current_value(self, location_id):
        return self.locations[location_id].get(location_id)