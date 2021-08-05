'''
Created on 06.06.2021

@author: Enzo Brancaccio
'''
import unittest
import math
import tracked.trackingHelpers as th
import tracked.expression as e

from rosslt_msgs.msg import Int32Tracked
from rosslt_msgs.msg import Location as rosLocationMsg
from std_msgs.msg import Header
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Time

from tracked.tracked import Tracked
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
        
        #self.testLocation3 = Location.withRossltMsg(self.testMsg)
        self.testLocation3 = Location(self.testMsg)
        
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
        
        #self.testLocationG3 = Location2(self.testMsg)
        self.testLocationG3 = Location2.withRossltMsg(self.testMsg)
        
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
        
    def testArithmeticIntMultiplyOptimization(self):
        self.a_iMop = Tracked(4, Location("foo", 24))
        self.x_iMop = Tracked(0, Location("bar", 24))
        
        self.assertEqual(self.a_iMop.value, 4, "int value")
        self.assertTrue(self.a_iMop.location_map["."].isValid(), "filled tracked Location")
        
        self.assertEqual(self.x_iMop.value, 0, "int value")
        self.assertTrue(self.x_iMop.location_map["."].isValid(), "filled tracked Location")
        
        self.b_iMop = self.a_iMop * 0
        
        self.assertEqual(self.b_iMop.value, 0, "int value")
        self.assertFalse(self.b_iMop.location_map["."].isValid(), "empty tracked Location")
        
        self.c_iMop = 0 * self.a_iMop
        
        self.assertEqual(self.c_iMop.value, 0, "int value")
        self.assertFalse(self.c_iMop.location_map["."].isValid(), "empty tracked Location")
        
        self.d_iMop = self.x_iMop * self.a_iMop
        
        self.assertEqual(self.d_iMop.value, 0, "int value")
        self.assertTrue(self.d_iMop.location_map["."].isValid(), "tracked Location")
        self.assertEqual(self.d_iMop.location_map["."].source_node, "bar", "source node")
        
        self.e_iMop = self.a_iMop * self.x_iMop
        
        self.assertEqual(self.e_iMop.value, 0, "int value")
        self.assertTrue(self.e_iMop.location_map["."].isValid(), "tracked Location")
        self.assertEqual(self.e_iMop.location_map["."].source_node, "bar", "source node")
        
    def testArithmeticIntDiv(self):
        self.a_iDiv = Tracked(4)
        
        self.assertEqual(self.a_iDiv.value, 4, "int value")
        
        self.b_iDiv = self.a_iDiv / 2
        
        self.assertEqual(self.b_iDiv.value, 2, "1st div")
        
        self.c_iDiv = 19 / self.a_iDiv
        
        self.assertEqual(self.c_iDiv.value, 4.75, "2nd div")
        
        self.d_iDiv = self.c_iDiv / self.b_iDiv
        
        self.assertEqual(self.d_iDiv.value, 2.375, "3rd div")
        self.assertEqual(self.d_iDiv.location_map["."].expression, "19;swap;/;2.0;/;", "expression")
        
    def testTrigonometry(self):
        self.a_Trig = Tracked(math.pi)
        
        self.b_Trig = Tracked.sin(self.a_Trig / 6.0)
        
        self.assertAlmostEqual(self.b_Trig.value, 0.5)
        self.assertEqual(self.b_Trig.location_map["."].expression, "6.0;/;sin;", "expression of sin")    
        
        self.c_Trig = Tracked.cos(self.a_Trig / 3.0)
        
        self.assertAlmostEqual(self.c_Trig.value, 0.5)
        self.assertEqual(self.c_Trig.location_map["."].expression, "3.0;/;cos;", "expression of cos")
        
    def testMakeTracked(self):
        self.assertEqual(th.make_tracked(5).value, 5, "simple make_tracked")
        self.assertEqual(th.make_tracked(th.make_tracked(5)).value, 5, "double make_tracked")
        
        self.assertTrue(th.make_tracked(False, Location("foo", 42)).location_map["."].isValid(), "location")
        self.assertFalse(th.make_tracked("Hallo").location_map["."].isValid())
        
    def testVectorMethods(self):
        self.sizeTest = Tracked([1, 2, 3])
        self.a_vm = Tracked(list())
        self.vm_loc1 = Location("foo", 22)
        self.vm_loc2 = Location("bar", 23)
        
        self.assertEqual(self.sizeTest.size(), 3, "size check  test")
        self.assertEqual(self.a_vm.size(), 0, "size check empty")
        
        self.a_vm.push_back(42)
        self.a_vm.push_back(th.make_tracked(7, self.vm_loc1))
        self.a_vm.push_back(-7)
        self.a_vm.push_back(th.make_tracked(15, self.vm_loc2))
        
        self.assertEqual(self.a_vm.size(), 4, "size check filled")
        
        self.assertEqual(self.a_vm.value[0], 42)
        self.assertEqual(self.a_vm.value[1], 7)
        self.assertEqual(self.a_vm.value[2], -7)
        self.assertEqual(self.a_vm.value[3], 15)
        self.assertEqual(self.a_vm.value[-1], 15)
        
        self.assertEqual(self.a_vm.front(), 42)
        self.assertEqual(self.a_vm.back(), 15)
        
        self.assertFalse(self.a_vm[0].location_map["."].isValid(), "no location")
        self.assertEqual(self.a_vm[1].location_map["."].location_id, 22, "1st location")
        self.assertEqual(self.a_vm[-1].location_map["."].location_id, 23, "2nd location")
        
        self.a_vm.pop_back()
        
        self.assertEqual(self.a_vm.size(), 3, "size check popped")
        self.assertEqual(self.a_vm[-1].value, -7)
        self.assertNotEqual(self.a_vm[-1].location_map["."].location_id, 23, "2nd location")
        
        self.a_vm.clear()
        
        self.assertEqual(self.a_vm.size(), 0, "size check cleared")
        
    def testApplyExpression(self):
        # numbers
        self.assertEqual(e.applyExpression(5, ""), 5)
        self.assertEqual(e.applyExpression(5, "\"1\";+;"), 6)

        self.assertEqual(e.applyExpression(4, "2;+;"), 6)
        self.assertEqual(e.applyExpression(0, "2;-;"), -2)
        self.assertEqual(e.applyExpression(3, "4;*;"), 12)
        self.assertEqual(e.applyExpression(15, "5;/;"), 3)

        self.assertEqual(e.applyExpression(3, "18;swap;/;"), 6)
        self.assertEqual(e.applyExpression(0, "2;swap;-;"), 2)

        self.assertEqual(e.applyExpression(4, "1;1;1;1;+;+;+;+;"), 8)
        self.assertEqual(e.applyExpression(1, "2;3;*;4;2;/;+;-;"), -7)

        self.assertEqual(e.applyExpression(1, "sin;"), math.sin(1))
        self.assertEqual(e.applyExpression(0, "cos;"), 1)
        
        self.assertEqual(e.applyExpression(5.0, ""), 5)
        self.assertEqual(e.applyExpression(5.0, "\"1.23\";+;"), 6.23)

        self.assertEqual(e.applyExpression(4.0, "2.5;+;"), 6.5)
        self.assertEqual(e.applyExpression(0.0, "2;-;"), -2)
        self.assertEqual(e.applyExpression(3.0, "4;*;"), 12)
        self.assertEqual(e.applyExpression(15.0, "6;/;"), 15.0/6.0)

        self.assertEqual(e.applyExpression(3.0, "18;swap;/;"), 6)
        self.assertEqual(e.applyExpression(0.0, "2;swap;-;"), 2)
        self.assertEqual(e.applyExpression(6.0, "4;swap;*;"), 24.0)
        self.assertEqual(e.applyExpression(2.0, "3;swap;+;"), 5.0)

        self.assertEqual(e.applyExpression(4.0, "1;1;1;1;+;+;+;+;"), 8)
        self.assertEqual(e.applyExpression(1.0, "2;3;*;4;2;/;+;-;"), -7)

        self.assertEqual(e.applyExpression(1.0, "sin;"), math.sin(1.0))
        self.assertEqual(e.applyExpression(4.0, "cos;"), math.cos(4.0))

        self.assertEqual(e.applyExpression(0.4, "asin;"), math.asin(0.4))
        self.assertEqual(e.applyExpression(0.3, "acos;"), math.acos(0.3))
        
        self.assertEqual(e.applyExpression(4.0, "\"2.5;+;"), 6.5)
        self.assertEqual(e.applyExpression(4.0, "2.5\";+;"), 6.5)
        self.assertEqual(e.applyExpression(4.0, "2\".5;+;"), 6.5)
        self.assertEqual(e.applyExpression(4.0, "\"2.5\";+;"), 6.5)
        
        # strings
        self.assertEqual(e.applyExpression("Hallo", ""), "Hallo")
        self.assertEqual(e.applyExpression("", "\"Hallo;Hallo;Hallo\";+;"), "Hallo;Hallo;Hallo")
        self.assertEqual(e.applyExpression("Hallo", "\" Welt\";+;"), "Hallo Welt")
        self.assertEqual(e.applyExpression("Hallo", "\" Welt\";swap;+;"), " WeltHallo")
        self.assertEqual(e.applyExpression("Hallo Welt", "\" Welt\";-;"), "Hallo")
        
    def testReverseExpression(self):
        self.assertEqual(e.reverseExpression(""), "");

        self.assertEqual(e.applyExpression(5.0, e.reverseExpression("")), 5)
        self.assertEqual(e.applyExpression(6.23, e.reverseExpression("\"1.23\";+;")), 5.0)

        self.assertEqual(e.applyExpression(6.5, e.reverseExpression("2.5;+;")), 4.0)
        self.assertEqual(e.applyExpression(-2.0, e.reverseExpression("2;-;")), 0)
        self.assertEqual(e.applyExpression(12.0, e.reverseExpression("4;*;")), 3.0)
        self.assertEqual(e.applyExpression(15.0/6.0, e.reverseExpression("6;/;")), 15.0)

        self.assertEqual(e.applyExpression(6.0, e.reverseExpression("18;swap;/;")), 3.0)
        self.assertEqual(e.applyExpression(2.0, e.reverseExpression("2;swap;-;")), 0.0)
        self.assertEqual(e.applyExpression(24.0, e.reverseExpression("4;swap;*;")), 6.0)
        self.assertEqual(e.applyExpression(5.0, e.reverseExpression("3;swap;+;")), 2.0)

        self.assertEqual(e.applyExpression(8.0, e.reverseExpression("1;1;1;1;+;+;+;+;")), 4.0)
        self.assertEqual(e.applyExpression(-7.0, e.reverseExpression("2;3;*;4;2;/;+;-;")), 1.0)

        self.assertEqual(e.applyExpression(math.sin(1.0), e.reverseExpression("sin;")), 1.0)
        self.assertEqual(e.applyExpression(math.cos(2.0), e.reverseExpression("cos;")), 2.0)

        self.assertEqual(e.applyExpression(1.2, e.reverseExpression("2.000000;+;10.000000;/;")), 10.0)
        
    def testSetPrimitiveField(self):
        self.spf_TrackedVM = Tracked(Marker)
        self.spf_loc = Location("foo", 22)
        
        self.spf_TrackedVM.id = th.make_tracked(42, self.spf_loc)
        
        self.assertEqual(self.spf_TrackedVM.id.location_map["."].location_id, 22)
        self.assertEqual(self.spf_TrackedVM.id.value, 42)
        
        self.spf_TrackedInt = self.spf_TrackedVM.id
        
        self.assertEqual(self.spf_TrackedInt.location_map["."].location_id, 22)
        self.assertEqual(self.spf_TrackedInt.value, 42)
        
        self.spf_TrackedVM.id = 25
        
        self.spf_TrackedInt = th.make_tracked(self.spf_TrackedVM.id)
        
        self.assertFalse(self.spf_TrackedInt.location_map["."].isValid())
        self.assertEqual(self.spf_TrackedInt.value, 25)
        
        self.spf_TrackedVM.text = "test"
        
        self.assertEqual(self.spf_TrackedVM.text, "test")
        
    def testSetComplexField(self):
        self.scf_TrackedVM = Tracked(Marker)
        self.scf_loc = Location("foo", 22)
        self.scf_head = Header()
        self.scf_time = Time()
        self.scf_time.nanosec = 12345
        
        self.scf_head.frame_id = "bar"
        self.scf_head.stamp = self.scf_time
        
        self.scf_TrackedVM.header = self.scf_head
        
        self.assertEqual(self.scf_TrackedVM.header.frame_id, "bar")
        self.assertFalse(self.scf_TrackedVM.location_map["."].isValid())
        
        self.scf_TrackedH = Tracked(Header)
        self.scf_TrackedH.value = self.scf_TrackedVM.header
        
        self.assertTrue(isinstance(self.scf_TrackedH, Tracked))
        self.assertFalse(self.scf_TrackedH.location_map["."].isValid())
        self.assertEqual(self.scf_TrackedH.stamp, self.scf_head.stamp)
        
        self.scf_head.frame_id = "baz"
        self.scf_time.nanosec = 10
        self.scf_head.stamp = self.scf_time

        self.scf_TrackedVM.header = th.make_tracked(self.scf_head, self.scf_loc)

        self.assertEqual(self.scf_TrackedVM.header.frame_id, "baz")
        self.assertFalse(self.scf_TrackedVM.location_map["."].isValid())

        self.scf_TrackedH = self.scf_TrackedVM.header

        self.assertTrue(self.scf_TrackedH.location_map["."].isValid())
        self.assertEqual(self.scf_TrackedH.stamp, self.scf_head.stamp)
        
    def testVectorIterator(self):
        self.vecit = Tracked([])
        self.vi_loc1 = Location("foo", 22)
        self.vi_loc2 = Location("bar", 23)

        self.assertEqual(self.vecit.size(), 0)

        self.vecit.push_back(42)
        self.vecit.push_back(th.make_tracked(7, self.vi_loc1))
        self.vecit.push_back(-7)
        self.vecit.push_back(th.make_tracked(15, self.vi_loc2))

        self.assertEqual(self.vecit.size(), 4)
        
        self.vecit_tracked1 = self.vecit[0]
        self.vecit_tracked2 = self.vecit[1]
        self.vecit_tracked3 = self.vecit[2]
        self.vecit_tracked4 = self.vecit[3]
        self.vecit_tracked5 = self.vecit[-1]
        
        self.assertEqual(self.vecit_tracked1.value, 42)
        self.assertEqual(self.vecit_tracked2.value, 7)
        self.assertEqual(self.vecit_tracked3.value, -7)
        self.assertEqual(self.vecit_tracked4.value, 15)
        self.assertEqual(self.vecit_tracked5.value, 15)

        self.assertFalse(self.vecit_tracked1.location_map["."].isValid())
        self.assertTrue(self.vecit_tracked2.location_map["."].isValid())
        self.assertFalse(self.vecit_tracked3.location_map["."].isValid())
        self.assertTrue(self.vecit_tracked4.location_map["."].isValid())
        self.assertTrue(self.vecit_tracked5.location_map["."].isValid())
        
        self.assertEqual(self.vecit_tracked2.location_map["."].location_id, 22, "1st location")
        self.assertEqual(self.vecit_tracked4.location_map["."].location_id, 23, "2nd location")
        self.assertEqual(self.vecit_tracked5.location_map["."].location_id, 23, "2nd location")
        
        for index in range(len(self.vecit.value)):
            self.vecit.value[index] = th.make_tracked(111, self.vi_loc2)
            
        for index in range(len(self.vecit.value)):
            with self.subTest(index = index):
                self.assertEqual(self.vecit[index].value, 111)
                self.assertEqual(self.vecit[index].location_map["."], self.vi_loc2)
                
        self.vecit.clear()
        
        self.assertEqual(self.vecit.size(), 0)
                
    def testSetArrayField(self):
        self.saf_TrackedVM = Tracked(Marker)
        self.saf_loc = Location("foo", 22)
        self.saf_col1 = ColorRGBA()
        self.saf_col2 = ColorRGBA()
        
        self.saf_col1.r = 0.5
        self.saf_col2.r = 0.2
        
        self.saf_trackedColors = Tracked(list())
        
        self.saf_trackedColors.push_back(self.saf_col1)
        self.saf_trackedColors.push_back(th.make_tracked(self.saf_col2, self.saf_loc))
        
        self.assertEqual(self.saf_trackedColors[0].r, 0.5)
        self.assertEqual(self.saf_trackedColors[1].r, 0.2)
        
        self.saf_TrackedVM.colors = self.saf_trackedColors
        
        self.assertEqual(len(self.saf_TrackedVM.colors.value), 2)
        self.assertEqual(self.saf_TrackedVM.colors[0].r, 0.5)
        self.assertEqual(self.saf_TrackedVM.colors[1].r, 0.2)
        
        self.assertFalse(self.saf_TrackedVM.colors.location_map["."].isValid())
        self.assertFalse(self.saf_TrackedVM.colors[0].location_map["."].isValid())
        self.assertTrue(self.saf_TrackedVM.colors[1].location_map["."].isValid())
        self.assertEqual(self.saf_TrackedVM.colors[1].location_map["."].location_id, 22)

if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    unittest.main()