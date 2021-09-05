'''
Created on 06.08.2021

@author: Enzo Brancaccio
'''

import inspect
import tracked.expression as e

from rclpy.node import Node
from .tracked import Tracked
from .locationManager import LocationManager
from .locationFunc import LocationFunc
from .location import Location

class TrackingNode(Node):
    '''
    classdocs
    '''


    def __init__(self, name):
        '''
        Constructor
        '''
        self.name = name
        self.loc_mgr = LocationManager(self)
        
    def loc(self, data, source_location = inspect.stack()):
        self.id = self.loc_mgr.get_location_id(source_location)
        self.loc_name = "loc" + str(self.id)
        
        if(self.id < 0):
            self.new_string = self.get_parameter(self.loc_name)
            self.loc_fun = LocationFunc(self.id, self.new_string)
            self.set_parameters(self.loc_name, self.loc_fun.new_value)
            self.id = self.loc_mgr.create_location(self.loc_fun, source_location)
            self.declare_parameter(self.loc_name, data)
        else:
            data = self.get_parameter(self.loc_name)
        
        self.location = Location(self.get_name(), self.id)
        return Tracked(data, self.location)
    
    def force_value(self, tracked_value, new_value):
        if(not tracked_value.location_map["."].isValid()):
            return
        else:
            self.new_expression = e.reverseExpression(tracked_value.location_map["."].expression)
            self.new_value_rev = e.applyExpression(Tracked(new_value), self.new_expression)
            
            self.source_node = tracked_value.location_map["."].source_node
            self.location_id = tracked_value.location_map["."].location_id
            self.updated_value = str(self.new_value_rev)
            self.loc_mgr.change_location(self.source_node, self.location_id, self.updated_value)
            
    def reevaluate(self, tracked_value):
        pass