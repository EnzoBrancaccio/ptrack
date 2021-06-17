'''
Created on 06.06.2021

@author: Enzo Brancaccio
'''
import unittest

from tracked.tracked import Tracked
from rosslt_msgs.msg import Int32Tracked
from rosslt_msgs.msg import Location as rosLocationMsg
from tracked.location import Location

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

    def testIntFromMsgCreation(self):
        self.msg = Int32Tracked
        self.msg.data = 7
        
        self.trackedTest2 = Tracked(self.msg.data)
        
        self.value = self.trackedTest2.value
        self.locationMap = self.trackedTest2.locationMap
        
        self.assertEqual(self.value, 7, "Value rosslt_msgs")
        self.assertFalse(self.locationMap["."].isValid(), "emptyLocation")

    def testLocation(self):
        self.testLocation1 = Location()
        
        self.assertFalse(self.testLocation1.isValid(), "empty Location")
        
        self.testLocation2 = Location("mynode", 42)
        
        self.assertTrue(self.testLocation2.isValid(), "filled Location")
        
        self.testMsg = rosLocationMsg()
        self.testMsg = self.testLocation2.makeRossltLocationMsg()
        
        self.assertEqual(self.testMsg.source_node, "mynode", "source node")
        self.assertEqual(self.testMsg.location_id, 42, "location_id")
        
        self.testLocation3 = Location.withRossltMsg(self.testMsg)
        
        self.assertEqual(self.testLocation3.source_node, "mynode", "source node")
        self.assertEqual(self.testLocation3.location_id, 42, "location_id")

if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    unittest.main()