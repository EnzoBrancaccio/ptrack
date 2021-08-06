'''
Created on 06.08.2021

@author: Enzo Brancaccio
'''

from rclpy.node import Node
from rosslt_msgs.msg import SourceChange
from .location import Location

class LocationManager(object):
    '''
    classdocs
    '''


    def __init__(self, params):
        '''
        Constructor
        '''
        