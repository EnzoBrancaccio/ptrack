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

if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    unittest.main()
