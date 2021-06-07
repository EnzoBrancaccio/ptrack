'''
Created on 06.06.2021

@author: Enzo Brancaccio
'''
import unittest

from tracked.tracked import Tracked

class Test(unittest.TestCase):


    def setUp(self):
        pass


    def tearDown(self):
        pass


    def testName(self):
        pass
    
    def testTrackedIntCreation(self):
        test1 = Tracked(5).value
        
        self.assertEqual(test1, 5, "Tracking")



if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    unittest.main()