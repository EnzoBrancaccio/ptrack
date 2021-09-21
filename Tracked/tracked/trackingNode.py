'''
Created on 06.08.2021

@author: Enzo Brancaccio
'''

import inspect
import tracked.expression as e
import tracked.trackingHelpers as th

from rclpy.node import Node
from .tracked import Tracked
from .locationManager import LocationManager
from .locationFunc import LocationFunc
from .location import Location
from rosslt_msgs.srv import GetValue
from rosslt_msgs.srv._get_value import GetValue_Request
#from rosidl_runtime_py import utilities

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
            
    def reevaluate(self, tracked_obj):
        # something like: if(utilities.message)
        self.isMessage = "TODO"
        if(self.isMessage):
            # use method reevaluate_complex_value()
            # will take Tracked.value, so call like reevaluate_complex_value(tracked_obj.value)
            pass
        else:
            self.tv_source_node = tracked_obj.location_map["."].source_node
            self.tv_location_id = tracked_obj.location_map["."].location_id
            if(not tracked_obj.location_map["."].isValid()):
                return tracked_obj
            if (self.tv_source_node == self.get_name()):
                tracked_obj.value = self.loc_mgr.current_value(self.tv_location_id)
            else:
                self.client = self.create_client(GetValue, self.tv_source_node + "/get_slt_value")
                self.client.wait_for_service(timeout_sec=1.0)
                
                self.request = GetValue_Request
                self.request.location_id(self.tv_location_id)
                
                self.response_future = self.client.call_async(self.request)
                self.response = th.get_future(self, self.response_future)
                
                if(self.response is not None):
                    tracked_obj.value = self.response.result
            
            tracked_obj.value = e.applyExpression(tracked_obj.value, tracked_obj.location_map["."].expression)
            return tracked_obj
        
    # parameter is Tracked object
    def reevaluate_complex_value(self, tracked_obj):
        # built-in primitive types according to http://wiki.ros.org/msg
        built_in_primitive_types = ["bool", "int", "long", "float", "rospy.Time", "rospy.Duration"]
        # using inspect.getmembers() because messages use slots
        # tv_attrs is a list
        tv_attrs = inspect.getmembers(tracked_obj.value)
        tv_fields = dict()
        for key, value in tv_attrs:
            if(key == "_fields_and_field_types"):
                tv_fields = value
        for key, value in tv_fields.items():
            if(value in built_in_primitive_types):
                # reevaluate value directly
                self.reevaluate(tracked_obj.value)
            else:
                # dig deeper
                pass
