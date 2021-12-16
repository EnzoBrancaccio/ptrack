'''
Created on 09.11.2021

@author: Enzo Brancaccio
'''

import unittest
import rclpy

from src.rosslt_msgs.msg import SourceChange

import src.ptracking.ptracking.trackingHelpers as th
import src.ptracking.ptracking.trackingNode as tn
import src.ptracking.ptracking.tracked as t

class Test(unittest.TestCase):


    def setUp(self):
        pass


    def tearDown(self):
        pass


    def testName(self):
        pass

    def testExtractFields(self):
        self.test_sc = SourceChange
        self.test_sc.location_id = 7
        self.test_sc.source_node = "foo"

        self.tracked = t.Tracked(self.test_sc)

        self.fields = th.extract_fields(self.tracked.value, "_fields_and_field_types")

        self.assertIn("location_id", self.fields)
        self.assertIn("source_node", self.fields)

    def testDocumentation(self):
        self.a = t.Tracked(5.0)
        self.b = self.a + 2.0

        self.assertEqual(self.b.value, 7.0)

        self.tmsg = t.Tracked(SourceChange)
        self.tmsg.source_node = "foo"
        self.name = self.tmsg.source_node

        self.assertEqual(self.tmsg.source_node, "foo")
        self.assertEqual(self.name, "foo")
        self.assertIsInstance(self.tmsg, t.Tracked)

if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    unittest.main()
