'''
Created on 10.06.2021

@author: Enzo Brancaccio
'''

import tracked.locationMap as lm
from tracked.location import Location

locationTest = Location("nodeTest", 1)
        
locationMapA1 = dict([("test1", locationTest)])  
locationMapA2 = dict([(".", locationTest)])
locationMapA3 = dict([("test3", locationTest)])
      
expectedA1 = dict([("test1", locationTest)])
expectedA2 = dict([("case2", locationTest)])
expectedA3 = dict([("case3/test3", locationTest)])

testlmA1 = dict()       
caseA1 = lm.addLocations(testlmA1, locationMapA1, "")
testlmA2 = dict()
caseA2 = lm.addLocations(testlmA2, locationMapA2, "case2")
testlmA3 = dict()
caseA3 = lm.addLocations(testlmA3, locationMapA3, "case3")

'''
print("Test case1")
for key, value in locationMapA1.items():
    print(key)
for key, value in expectedA1.items():
    print(key)
for key, value in caseA1.items():
    print(key)
       
print("Test case2")
for key, value in locationMapA2.items():
    print(key)
for key, value in expectedA2.items():
    print(key)
for key, value in caseA2.items():
    print(key)
        
print("Test case3")  
for key, value in locationMapA3.items():
    print(key)    
for key, value in expectedA3.items():
    print(key)   
for key, value in caseA3.items():
    print(key)
'''
    
locationTestR1 = Location("testNode1", 1)
locationTestR2 = Location("testNode2", 2)
        
locationMapR1 = dict([("test1", locationTestR1), ("posTest", locationTestR2)])
locationMapR2 = dict([("test1", locationTestR1), ("negTest", locationTestR2)])
    
expectedR1 = dict([("test1", locationTestR1)])
expectedR2 = dict([("test1", locationTestR1), ("negTest", locationTestR2)])
    
caseR1 = lm.removeLocations(locationMapR1, "pos")
caseR2 = lm.removeLocations(locationMapR2, "pos")

'''
print("Test case1")
for key, value in locationMapR1.items():
    print(key)
for key, value in expectedR1.items():
    print(key)
for key, value in caseR1.items():
    print(key)
       
print("Test case2")
for key, value in locationMapR2.items():
    print(key)
for key, value in expectedR2.items():
    print(key)
for key, value in caseR2.items():
    print(key)
'''

locationTest = Location("nodeTest", 1)
        
locationMapS1 = dict([("no/no/no", locationTest)])
locationMapS2 = dict([("yes", locationTest)])
locationMapS3 = dict([("yes/no", locationTest)])
        
expectedS1 = dict()
expectedS2 = dict([(".", locationTest)])
expectedS3 = dict([("no", locationTest)])
        
caseS1 = lm.locationSlice(locationMapS1, "yes")
caseS2 = lm.locationSlice(locationMapS2, "yes")       
caseS3 = lm.locationSlice(locationMapS3, "yes")
        
print("Test case1")
for key, value in locationMapS1.items():
    print(key)
for key, value in expectedS1.items():
    print(key)
for key, value in caseS1.items():
    print(key)
       
print("Test case2")
for key, value in locationMapS2.items():
    print(key)
for key, value in expectedS2.items():
    print(key)
for key, value in caseS2.items():
    print(key)
        
print("Test case3")  
for key, value in locationMapS3.items():
    print(key)    
for key, value in expectedS3.items():
    print(key)   
for key, value in caseS3.items():
    print(key)