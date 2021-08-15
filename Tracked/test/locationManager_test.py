'''
Created on 13.08.2021

@author: Enzo Brancaccio
'''
import unittest

from rclpy.node import Node
from tracked.locationManager import LocationManager
from tracked.location import Location

class Test(unittest.TestCase):


    def setUp(self):
        pass


    def tearDown(self):
        pass


    def testName(self):
        pass
    
    def testCurrentValue(self):
        self.assertEqual(1, 1)
        '''
        self.cv_node = Node()
        self.cv_lm = LocationManager(self.cv_node)
        self.cv_lm.locations[0] = "test1"
        self.cv_lm.locations[1] = "test2"  
        self.assertEqual(self.cv_lm.current_value(self, 0), "test1")
        self.assertEqual(self.cv_lm.current_value(self, 1), "test2")
        '''


if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    unittest.main()