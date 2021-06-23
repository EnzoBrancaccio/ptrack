'''
Created on 06.06.2021

@author: Enzo Brancaccio
'''
import unittest

from tracked.tracked import Tracked
from rosslt_msgs.msg import Int32Tracked
from rosslt_msgs.msg import Location as rosLocationMsg
from tracked.location import Location
from tracked.tracked2 import Tracked2
from tracked.location2 import Location2

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
        self.location_map = self.trackedTest1.location_map
        
        self.assertEqual(self.value, 5, "Value")
        self.assertFalse(self.location_map["."].isValid(), "emptyLocation")
        
        # variables with G ("Generic") are with Tracked2 or Location2
        self.trackedTestG = Tracked2(5)
        
        self.valueG = self.trackedTestG.value
        self.location_mapG = self.trackedTestG.location_map
        
        self.assertEqual(self.valueG, 5, "Value")
        self.assertFalse(self.location_mapG["."].isValid(), "emptyLocation")

    def testIntFromMsgCreation(self):
        self.msg = Int32Tracked
        self.msg.data = 7
        
        self.trackedTest2 = Tracked(self.msg.data)
        
        self.value = self.trackedTest2.value
        self.location_map = self.trackedTest2.location_map
        
        self.assertEqual(self.value, 7, "Value rosslt_msgs")
        self.assertFalse(self.location_map["."].isValid(), "emptyLocation")
        
        self.trackedTestG = Tracked2(self.msg.data)
        
        self.valueG = self.trackedTestG.value
        self.location_mapG = self.trackedTestG.location_map
        
        self.assertEqual(self.valueG, 7, "Value rosslt_msgs")
        self.assertFalse(self.location_mapG["."].isValid(), "emptyLocation")

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
        
        self.testLocationG = Location2()
        
        self.assertFalse(self.testLocationG.isValid(), "empty Location")
        
        self.testLocationG2 = Location2("mynode", 42)
        
        self.assertTrue(self.testLocationG2.isValid(), "filled Location")
        
        self.testMsgG = rosLocationMsg()
        self.testMsgG = self.testLocationG2.makeRossltLocationMsg()
        
        self.assertEqual(self.testMsgG.source_node, "mynode", "source node")
        self.assertEqual(self.testMsgG.location_id, 42, "location_id")
        
        self.testLocationG3 = Location2(self.testMsg)
        
        self.assertEqual(self.testLocationG3.source_node, "mynode", "source node")
        self.assertEqual(self.testLocationG3.location_id, 42, "location_id")

    def testTrackedWithLocation(self):
        self.trackedLocation1 = Location("mynode", 42)
        
        self.assertTrue(self.trackedLocation1.isValid(), "filled tracked Location")
        
        self.trackedDouble = Tracked(2.0, self.trackedLocation1)
        
        self.trackedDoubleLocation = self.trackedDouble.location_map["."]
        self.trackedDoubleLocationId = self.trackedDoubleLocation.location_id
        self.assertEqual(self.trackedDoubleLocationId, 42, "tracked location id")
        
        self.newTrackedDouble = self.trackedDouble
        
        self.newTrackedDoubleLocation = self.newTrackedDouble.location_map["."]
        self.newTrackedDoubleLocationId = self.newTrackedDoubleLocation.location_id
        self.assertEqual(self.newTrackedDoubleLocationId, 42, "tracked location id")
        
        self.trackedLocationG = Location2("mynode", 42)
        
        self.assertTrue(self.trackedLocationG.isValid(), "filled tracked Location")
        
        self.trackedDoubleG = Tracked2(2.0, self.trackedLocationG)
        
        self.trackedDoubleLocationG = self.trackedDoubleG.location_map["."]
        self.trackedDoubleLocationIdG = self.trackedDoubleLocationG.location_id
        self.assertEqual(self.trackedDoubleLocationIdG, 42, "tracked location id")
        
        self.newTrackedDoubleG = self.trackedDoubleG
        
        self.newTrackedDoubleLocationG = self.newTrackedDoubleG.location_map["."]
        self.newTrackedDoubleLocationIdG = self.newTrackedDoubleLocationG.location_id
        self.assertEqual(self.newTrackedDoubleLocationIdG, 42, "tracked location id")
        
    def testArithmeticDoublePlus(self):
        self.a_adp = Tracked(5.0)
        
        self.assertEqual(self.a_adp.value, 5.0, "hold value")
        
        self.b_adp = self.a_adp + 2.0
        
        self.assertEqual(self.b_adp.value, 7.0, "hold new value")
        self.assertEqual(self.a_adp.value, 5.0, "still hold value")
        
        self.c_adp = 3.0 + self.a_adp
        
        self.assertEqual(self.c_adp.value, 8.0, "hold new value")
        self.assertEqual(self.c_adp.location_map["."].expression, "3.0;swap;+;")
        
        self.d_adp = self.b_adp + self.c_adp
        
        self.assertEqual(self.d_adp.value, 15.0, "hold new value")
        self.assertEqual(self.d_adp.location_map["."].expression, "2.0;+;8.0;+;")
        
    def testArithmeticString(self):
        self.astring = Tracked("Hallo")
        
        self.assertEqual(self.astring.value, "Hallo", "String comparison")
        
        self.bstring = "Oh, " + self.astring + " Welt";
        
        self.assertEqual(self.bstring.value, "Oh, Hallo Welt", "Hello World")
        
    def testArithmeticMinus(self):
        self.aminus = Tracked(4)
        
        self.assertEqual(self.aminus.value, 4, "initial value")
        
        self.bminus = self.aminus - 3
        
        self.assertEqual(self.bminus.value, 1, "1st sub")
        
        self.cminus = 3 - self.aminus
        
        self.assertEqual(self.cminus.value, -1, "2nd sub")
        self.assertEqual(self.cminus.location_map["."].expression, "3;swap;-;")
        
        self.dminus = self.cminus - self.bminus
        self.assertEqual(self.dminus.location_map["."].expression, "3;swap;-;1;-;")
        
    def testArithmeticIntMultiply(self):
        self.a_iMul = Tracked(4, Location("foo", 27))
        
        self.assertEqual(self.a_iMul.value, 4, "int value")
        
        self.b_iMul = self.a_iMul * 3
        
        self.assertEqual(self.b_iMul.value, 12, "1st mul")
        
        self.c_iMul = 2 * self.b_iMul
        
        self.assertEqual(self.c_iMul.value, 24, "2nd mul")
        
        self.d_iMul = self.c_iMul * self.a_iMul
        
        self.assertEqual(self.d_iMul.value, 96, "3rd mul")
        self.assertEqual(self.d_iMul.location_map["."].expression, "3;*;2;swap;*;4;*;")      

if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    unittest.main()