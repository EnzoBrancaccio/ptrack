'''
Created on 06.08.2021

@author: Enzo Brancaccio
'''

import inspect
import src.ptracking.ptracking.expression as e
import src.ptracking.ptracking.trackingHelpers as th

from rclpy.node import Node
from src.ptracking.ptracking.tracked import Tracked
from src.ptracking.ptracking.locationManager import LocationManager
from src.ptracking.ptracking.locationFunc import LocationFunc
from src.ptracking.ptracking.location import Location
from src.rosslt_msgs.srv import GetValue
from src.rosslt_msgs.srv._get_value import GetValue_Request
from rosidl_runtime_py import utilities

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
    
    # unfinished and will possibly never be used
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
        self.isMessage = utilities.is_message(tracked_obj.value)
        if(self.isMessage):
            self.reevaluate_msg(tracked_obj)
            return tracked_obj
        else:
            return self.reevaluate_value(tracked_obj)
        
    # parameter is Tracked object
    # also see http://wiki.ros.org/msg
    def reevaluate_msg(self, tracked_obj):
        to_fields = th.extract_fields(tracked_obj.value, "_fields_and_field_types")
        tracked_obj.value = self.reevaluate_generic(tracked_obj, to_fields)
        return tracked_obj

    # first try: act as if the sub-Tracked.value values were not Tracked
    # obj = tracked_obj.value
    def reevaluate_submsg(self, obj, ex_fieldname):
        field = getattr(obj, ex_fieldname)
        field_fields = th.extract(field, "_fields_and_field_types")
        field = self.reevaluate_generic(field, field_fields)
        setattr(obj, ex_fieldname, field)
        return obj

    def reevaluate_generic(self, obj, fields):
        # key: fieldname, value: fieldtype in Python
        for fieldname, fieldtype in fields.items():
            if(hasattr(obj, fieldname)):
                # check this value (fieldname of Tracked.value):
                sub_field = getattr(obj, fieldname)
                # if message -> dig deeper, else -> reevaluate value
                if(utilities.is_message(sub_field)):
                    obj = self.reevaluate_submsg(obj, fieldname)
                else:
                    # reevaluate value directly and update tracked object's value
                    obj = self.reevaluate_value(obj, fieldname)
        return obj

        
    # outsourcing update of value to also use it inside reevaluate_msg
    # either update Tracked.value directly
    # or via fieldname with setattr ("value" is default for ease of use)     
    def reevaluate_value(self, tracked_obj, fieldname = "value"):
        self.to_source_node = tracked_obj.location_map["."].source_node
        self.to_location_id = tracked_obj.location_map["."].location_id
        if(not tracked_obj.location_map["."].isValid()):
            return tracked_obj
        if (self.to_source_node == self.get_name()):
            setattr(tracked_obj, fieldname, self.loc_mgr.current_value(self.to_location_id))
        else:
            self.client = self.create_client(GetValue, self.to_source_node + "/get_slt_value")
            self.client.wait_for_service(timeout_sec=1.0)
                
            self.request = GetValue_Request
            self.request.location_id(self.to_location_id)
                
            self.response_future = self.client.call_async(self.request)
            self.response = th.get_future(self, self.response_future)
                
            if(self.response is not None):
                setattr(tracked_obj, fieldname, self.response.result)
            
        setattr(tracked_obj, fieldname, e.applyExpression(tracked_obj.value, tracked_obj.location_map["."].expression))
        return tracked_obj
