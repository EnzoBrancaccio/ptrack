'''
Created on 10.06.2021

@author: Enzo Brancaccio
'''

import math
import tracked.locationMap as lm
from tracked.location import Location
from tracked.tracked import Tracked
from rosslt_msgs.msg import Location as rosLocationMsg
from tracked.tracked2 import Tracked2
from tracked.location2 import Location2

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

'''        
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
'''

testLocation1 = Location()
        
testLocation2 = Location("mynode", 42)
        
testMsg = rosLocationMsg()
testMsg = testLocation2.makeRossltLocationMsg()
        
testLocation3 = Location.withRossltMsg(testMsg)

'''
print("testLocation1")
print(testLocation1.source_node)
print(testLocation1.location_id)

print("testLocation2")
print(testLocation2.source_node)
print(testLocation2.location_id)

print("testLocation3")
print(testLocation3.source_node)
print(testLocation3.location_id)
'''

trackedLocationG = Location2("mynode", 42)
        
trackedDoubleG = Tracked2(2.0, trackedLocationG)
        
trackedDoubleLocationG = trackedDoubleG.location_map["."]
trackedDoubleLocationIdG = trackedDoubleLocationG.location_id
        
newTrackedDoubleG = trackedDoubleG
        
newTrackedDoubleLocationG = newTrackedDoubleG.location_map["."]
newTrackedDoubleLocationIdG = newTrackedDoubleLocationG.location_id

'''
print("trackedDoubleG")
print(trackedDoubleG.value)
print(trackedDoubleG.location.source_node)
print(trackedDoubleG.location.location_id)
print(trackedDoubleG.location_map)
print(trackedDoubleLocationG)
print(trackedDoubleLocationG.source_node)
print(trackedDoubleLocationG.location_id)
print(trackedDoubleLocationIdG)

print("trackedLocationG")
print(trackedLocationG.source_node)
print(trackedLocationG.location_id)
print(trackedDoubleLocationG.source_node)
print(trackedDoubleLocationG.location_id)
print(trackedDoubleLocationIdG)
print(newTrackedDoubleLocationG.source_node)
print(newTrackedDoubleLocationG.location_id)
print(newTrackedDoubleLocationIdG)
'''

a_adp = Tracked(5.0)
b_adp = Tracked(a_adp.value + 2.0)
c_adp = Tracked(3.0 + a_adp.value) 
d_adp = Tracked(b_adp.value + c_adp.value)

'''
print("values")
print(a_adp.value)
print(b_adp.value)
print(c_adp.value)
print(d_adp.value)

print("expressions")
print(a_adp.location_map["."].expression)
print(b_adp.location_map["."].expression)
print(c_adp.location_map["."].expression)
print(d_adp.location_map["."].expression)
'''

a2_adp = Tracked(5.0)
b2_adp = a2_adp + 2.0
c2_adp = 3.0 + a2_adp
d2_adp = a2_adp + b2_adp

'''
print("values2")
print(a2_adp.value)
print(b2_adp.value)
print(c2_adp.value)
print(d2_adp.value)

print("expressions2")
print(a2_adp.location_map["."].expression)
print(b2_adp.location_map["."].expression)
print(c2_adp.location_map["."].expression)
print(d2_adp.location_map["."].expression)
'''

astring = Tracked("Hallo")
bstring = "Oh, " + astring + " Welt";

'''
print("Strings")
print(astring.value)
print(bstring.value)
'''

a_Trig = Tracked(math.pi)    
b_Trig = Tracked.sin(a_Trig / 6.0)         
c_Trig = Tracked.cos(a_Trig / 3.0)

'''
print("sin/cos")
print(a_Trig.value)
print(b_Trig.value)
print(type(b_Trig.value))
print(b_Trig.location_map["."].expression)
print(c_Trig.location_map["."].expression)
'''

sizeTest = Tracked([1, 2, 3])
a_vm = Tracked(list())
vm_loc1 = Location("foo", 22)
vm_loc2 = Location("bar", 23)

print(isinstance(sizeTest.value, list))
print(isinstance(a_vm.value, list))
print(type(sizeTest.value))
print(type(a_vm.value))