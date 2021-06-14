'''
Created on 06.06.2021

@author: Enzo Brancaccio
'''
import unittest

from tracked.tracked import Tracked
from rosslt_msgs.msg import Int32Tracked

class Test(unittest.TestCase):


    def setUp(self):
        pass


    def tearDown(self):
        pass


    def testName(self):
        pass
    
    def testTrackedIntCreation(self):
        self.trackedTest1 = Tracked(5)
        
        self.value = self.trackedTest1.value
        self.locationMap = self.trackedTest1.locationMap
        
        self.assertEqual(self.value, 5, "Value")
        self.assertFalse(self.locationMap["."].isValid(), "emptyLocation")

if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    unittest.main()