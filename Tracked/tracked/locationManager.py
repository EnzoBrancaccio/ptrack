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
        self.sc_sub = node.create_subscription(SourceChange, "/sc") # callback?
        self.sc_pub = node.create_publisher(SourceChange, "/sc", 10)
        self.get_value_service = node.create_service(GetValue, node.get_name() + "/get_slt_value")