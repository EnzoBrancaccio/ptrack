'''
Created on 06.08.2021

@author: Enzo Brancaccio
'''

import rclpy
import inspect

from rclpy.node import Node
from src.rosslt_msgs.msg import SourceChange
from src.rosslt_msgs.srv import GetValue
from src.ptracking.ptracking.location import Location
from src.ptracking.ptracking.locationFunc import LocationFunc

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
        # {scource_location: int}, note that source_location is a named tuple
        self.source_locations = dict()
        # replaces C++ locations, list of LocationFunc instances
        self.locations = list()
        self.get_value_service = None
        self.node = Node
        
        # wondering where the msg for the placeholders come from
        self.sc_sub = node.create_subscription(SourceChange, "/sc", self.on_source_change, 10)
        self.sc_pub = node.create_publisher(SourceChange, "/sc", 10)
        self.get_value_service = node.create_service(GetValue, node.get_name() + "/get_slt_value", self.on_get_value, 10)
        
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
                
    def create_location(self, location_func, source_location = inspect.stack()):
        # source_locations is dict with key: source_location, value: int
        self.it = self.source_locations[source_location]
        if(self.it == list(self.source_locations.keys())[-1]):
            self.locations.append(location_func)
            self.source_locations[source_location] = len(self.locations) - 1
            return len(self.locations) - 1
        else:
            return self.it[source_location]
    
    def get_location_id(self, source_location):
        # source_locations is dict with key: source_location, value: int
        self.it = self.source_locations[source_location]
        if(self.it != list(self.source_locations.keys())[-1]):
            return self.it[source_location]
        else:
            return -1
    
    def change_location(self, source_node, location_id, new_value):
        self.msg = SourceChange
        self.msg.source_node(source_node)
        self.msg.location_id(location_id)
        self.msg.new_value(new_value)
        
        if(source_node == self.node.get_name()):
            self.on_source_change(self.msg)
        else:
            self.sc_pub.publish(self.msg)
    
    # self.locations is list of locationFunc instances        
    def current_value(self, location_id):
        return self.locations[location_id].get_value(location_id)