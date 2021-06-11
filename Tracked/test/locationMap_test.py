'''
Created on 09.06.2021

@author: Enzo Brancaccio
'''
import unittest

import tracked.locationMap as lm
from tracked.location import Location

class Test(unittest.TestCase): 

    def setUp(self):
        pass


    def tearDown(self):
        pass

    # case 1: prefix = ""
    # case 2: prefix = "case2"
    # case 3: prefix = "case3"
    def testAddLocations(self):
        locationTest = Location("nodeTest", 1)
        
        locationMap1 = dict([("test1", locationTest)])
        locationMap2 = dict([(".", locationTest)])
        locationMap3 = dict([("test3", locationTest)])
        
        testlm1 = dict()
        testlm2 = dict()
        testlm3 = dict()
        
        expected1 = dict([("test1", locationTest)])
        expected2 = dict([("case2", locationTest)])
        expected3 = dict([("case3/test3", locationTest)])
        
        case1 = lm.addLocations(testlm1, locationMap1, "")
        case2 = lm.addLocations(testlm2, locationMap2, "case2")       
        case3 = lm.addLocations(testlm3, locationMap3, "case3")
        
        self.assertCountEqual(case1, expected1, "addLocationsT1")
        self.assertCountEqual(case2, expected2, "addLocationsT2")
        self.assertCountEqual(case3, expected3, "addLocationsT3")

    # case 1: prefix found and entry removed
    # case 2: prefix not found
    def testRemoveLocations(self):
        locationTest1 = Location("testNode1", 1)
        locationTest2 = Location("testNode2", 2)
        
        locationMap1 = dict([("test1", locationTest1), ("posTest", locationTest2)])
        locationMap2 = dict([("test1", locationTest1), ("negTest", locationTest2)])
    
        expected1 = dict([("test1", locationTest1)])
        expected2 = dict([("test1", locationTest1), ("negTest", locationTest2)])
    
        case1 = lm.removeLocations(locationMap1, "pos")
        case2 = lm.removeLocations(locationMap2, "pos")
        
        self.assertCountEqual(case1, expected1, "removeLocationsT1")
        self.assertCountEqual(case2, expected2, "removeLocationsT2")
    
    # case 1: doesn't start with prefix
    # case 2: "."
    # case 3: substring
    def testSliceLocations(self):
        locationTest = Location("nodeTest", 1)
        
        locationMap1 = dict([("no/no/no", locationTest)])
        locationMap2 = dict([("yes", locationTest)])
        locationMap3 = dict([("yes/no", locationTest)])
        
        expected1 = dict()
        expected2 = dict([(".", locationTest)])
        expected3 = dict([("no", locationTest)])
        
        case1 = lm.locationSlice(locationMap1, "yes")
        case2 = lm.locationSlice(locationMap2, "yes")       
        case3 = lm.locationSlice(locationMap3, "yes")
        
        self.assertCountEqual(case1, expected1, "locationSliceT1")
        self.assertCountEqual(case2, expected2, "locationSliceT2")
        self.assertCountEqual(case3, expected3, "locationSliceT3")


if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    unittest.main()