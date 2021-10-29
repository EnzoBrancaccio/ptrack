'''
Created on 10.06.2021

@author: Enzo Brancaccio
'''

import math
import copy
import inspect

import src.ptracking.ptracking.locationMap as lm
import src.ptracking.ptracking.trackingHelpers as th
import src.ptracking.ptracking.expression as e

from builtin_interfaces.msg import Time

from src.ptracking.ptracking.location import Location
from src.ptracking.ptracking.tracked import Tracked
from src.rosslt_msgs.msg import Location as rosLocationMsg
#from tracked.tracked2 import Tracked2
#from tracked.location2 import Location2
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

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

'''
print("testLocation1")
print(testLocation1.source_node)
print(testLocation1.location_id)

print("testLocation2")
print(testLocation2.source_node)
print(testLocation2.location_id)
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

'''
print(isinstance(sizeTest.value, list))
print(isinstance(a_vm.value, list))
print(type(sizeTest.value))
print(type(a_vm.value))
'''

ae_val = 5
ae_expr = "\"1\";+;"
ae_stack = list()
ae_stack.append(ae_val)
ae_eList = ae_expr.split(";")
'''
for x in ae_eList:
    print(x)
print(ae_eList[0])
ae_quotList = ae_eList[0].split('\"')
for y in ae_quotList:
    print(y)
ae_token = ae_eList[0]
if(ae_token[0] == '"'):
    if(ae_token[-1] == '"'):
        print(ae_token)
        ae_newVal = ae_token.split('\"')[1]
        print(ae_newVal)
'''
ae_strexp = "\"Hallo;Hallo;Hallo\";+;"
ae_seList = ae_strexp.split(";")
for subToken in ae_seList:
    if(subToken != ""):
        if(subToken[0] == '"'):
            firstIndex = ae_seList.index(subToken)
        if(subToken[-1] == '"'):
            lastIndex = ae_seList.index(subToken)
ae_subList = ae_seList[firstIndex:(lastIndex+1)]
'''
for x in ae_subList:
    print(x)
ae_newValue = ";".join(ae_subList)
print(ae_newValue)
'''

re_expr = "\"1.23\";+;"
re_expList = re_expr.split(";")
n = 0
i = 0
for x in range(0, len(re_expList)):
    n = n - th.is_operator(re_expList[x])
    if(n < 0):
        break
    else:
        n = n + 1
        i = i + 1
'''
print(len(re_expList))
for rex in re_expList:
    print(rex)
print(n)
print(i)
'''
re_expResultList = list()
for y in range(0, i):
    re_expResultList.append(re_expList[y])
'''
for rey in re_expResultList:
    print(rey)
'''
re_is_rhs = (((i - 1) >= 0) and (re_expList[i-1] == "swap"))
re_is_commutative = (re_expList[i] == "+") or (re_expList[i] == "*")
'''
print(re_is_rhs)
print(re_is_commutative)
'''

'''
setat_tracked = Tracked([1, 3, 5])
print("setat_tracked before")
print(setat_tracked.value[0])
print("setat_tracked after")
setat_tracked.value[0] = 7
print(setat_tracked.value[0])
'''

'''
print("Tracked setitem tests")
tsi_1 = Tracked([2, 8, 9])
tsi_1[0] = 5
tsi_1[1] = 8
tsi_1[2] = "Foo"
print("content")
for index, value in enumerate(tsi_1):
    print(index, value)
'''

'''
vecit = Tracked([])
print("round 1")
for i in vecit.location_map.items():
    print(i)
vi_loc1 = Location("foo", 22);
vi_loc2 = Location("bar", 23);
vecit.push_back(42);
print("round 2")
for i in vecit.location_map.items():
    print(i)
vecit.push_back(th.make_tracked(7, vi_loc1));
print("round 3")
for i in vecit.location_map.items():
    print(i)
vecit.push_back(-7);
print("round 4")
for i in vecit.location_map.items():
    print(i)
vecit.push_back(th.make_tracked(15, vi_loc2));
print("round 5")
for i in vecit.location_map.items():
    print(i)
'''

'''
vecit = Tracked([])
vi_loc1 = Location("foo", 22)
vi_loc2 = Location("bar", 23)

vecit.push_back(42)
print("item 1")
for i in vecit:
    print(i)
    print(i.value)
    print(i.location_map["."].isValid())
    for j in i.location_map.items():
        print(j)
vecit.push_back(th.make_tracked(7, vi_loc1))
print("item 2")
for i in vecit:
    print(i)
    print(i.value)
    print(i.location_map["."].isValid())
    for j in i.location_map.items():
        print(j)
vecit.push_back(-7)
print("item 3")
for i in vecit:
    print(i)
    print(i.value)
    print(i.location_map["."].isValid())
    for j in i.location_map.items():
        print(j)
vecit.push_back(th.make_tracked(15, vi_loc2))
print("item 4")
for i in vecit:
    print(i)
    print(i.value)
    print(i.location_map["."].isValid())
    for j in i.location_map.items():
        print(j)

vecit_tracked1 = vecit[0]
vecit_tracked2 = vecit[1]
vecit_tracked3 = vecit[2]
vecit_tracked4 = vecit[3]

print("vecit_tracked1")
print(vecit_tracked1.value)
print(vecit_tracked1.location_map["."].isValid())
print("vecit_tracked2")
print(vecit_tracked2.value)
print(vecit_tracked2.location_map["."].isValid())
print("vecit_tracked3")
print(vecit_tracked3.value)
print(vecit_tracked3.location_map["."].isValid())
print("vecit_tracked4")
print(vecit_tracked4.value)
print(vecit_tracked4.location_map["."].isValid())
'''

# will no longer work (respective code deleted):
'''
dotTracked1 = Tracked(5)
print(dotTracked1.value)
dotTracked1.foo
dotVariable1 = dotTracked1.foo
print(dotVariable1)
dotTracked1.bar
dotTracked1.foot
print(dotTracked1.value)
dotVariable2 = dotTracked1.foo
print(dotVariable2)
dotTracked1.setToValue_6
print(dotTracked1.value)
'''

dotTracked2 = Tracked(5)
#dotTracked2.header

# doesn't work even without modified getattr (source_node not found)
# just to check if getattr-overloading is necessary
'''
msgTestLocation = Location("msgTest", 24)
msgTestLocationMsg = msgTestLocation.makeRossltLocationMsg()
msgTestTracked = Tracked(msgTestLocationMsg)
# this works, so only works with .value
print(msgTestTracked.value.source_node)
# this can only work with overloading getattr
print(msgTestTracked.source_node)
'''

'''
msgAttrTestLocation = Location("attrTest", 17)
msgAttrTestLocationMsg = msgAttrTestLocation.makeRossltLocationMsg()
msgAttrTestTracked = Tracked(msgAttrTestLocationMsg)
print(msgAttrTestTracked.value.source_node)
print(msgAttrTestTracked.source_node)
print(msgAttrTestTracked.location_id)
msgAttrTestTracked.source_node = "SET_FIELD Test"
print(msgAttrTestTracked.source_node)
'''

spf_TrackedVM = Tracked(Marker)
spf_loc = Location("foo", 22)

'''
print("before")
print(spf_TrackedVM.id)
spf_TrackedVM.id = th.make_tracked(42, spf_loc)
print("after")
print(spf_TrackedVM.id)
print(spf_TrackedVM.id.location_map)
print(spf_TrackedVM.id.location_map["."])
print(spf_TrackedVM.id.location_map["."].location_id)
print(spf_TrackedVM.id.value)
print("examination")
print(spf_TrackedVM)
print(spf_TrackedVM.value)
print(spf_TrackedVM.location_map)
print(spf_TrackedVM.location_map["."])
print(spf_TrackedVM.location_map["."].location_id)

print("TrackedInt Test")
print(spf_TrackedVM.id)
spf_TrackedInt = spf_TrackedVM.id
print(spf_TrackedInt)
print(spf_TrackedInt.value)
print(spf_TrackedInt.location_map)
print(spf_TrackedInt.location_map["."])
print(spf_TrackedInt.location_map["."].location_id)
'''

'''
scf_time = Time()
print(scf_time)
scf_time.nanosec = 12345
print(scf_time)
'''

'''
saf_TrackedVM = Tracked(Marker)
saf_loc = Location("foo", 22)
saf_col1 = ColorRGBA()
saf_col2 = ColorRGBA()
        
saf_col1.r = 0.5
saf_col2.r = 0.2

print(saf_TrackedVM.colors)
print(type(saf_TrackedVM.colors))

saf_trackedColors = Tracked(list())
#saf_trackedColors.value = saf_TrackedVM.colors
saf_trackedColors2 = Tracked(saf_TrackedVM.colors)
print(type(saf_trackedColors.value))
print(type(saf_trackedColors2.value))
        
saf_trackedColors.push_back(saf_col1)
saf_trackedColors.push_back(th.make_tracked(saf_col2, saf_loc))
    
print(saf_trackedColors[0])
print(saf_trackedColors[1])    
print(saf_trackedColors[0].r)
print(saf_trackedColors[1].r)

print("pure marker info")
saf_marker = Marker
print(type(saf_marker))
print(type(saf_marker.colors))
saf_marker.colors = 25
print(saf_marker.colors)
saf_marker.colors = 26
print(saf_marker.colors)
'''

'''
vecit = Tracked([])
vi_loc1 = Location("foo", 22)
vi_loc2 = Location("bar", 23)

vecit.push_back(42)
vecit.push_back(th.make_tracked(7, vi_loc1))
vecit.push_back(-7)
vecit.push_back(th.make_tracked(15, vi_loc2))

for i in vecit.value:
    print(i)
    
for index in range(len(vecit.value)):
    print(vecit.value[index])
    vecit.value[index] = vecit.value[index] + 1
    print(vecit.value[index])
    
for i in vecit.value:
    print(i)
'''

i_loc = Location("foo", 22)
i_members = inspect.getmembers(i_loc)
i_membmet = inspect.getmembers(i_loc, inspect.ismethod)
i_membfun = inspect.getmembers(i_loc, inspect.isfunction)
i_funcg = inspect.getfullargspec(type(i_loc).isValid)
i_funcs = inspect.signature(type(i_loc).isValid)
i_sfile = inspect.getsourcefile(type(i_loc))
i_file = inspect.getfile(type(i_loc))
i_source = inspect.getsource(type(i_loc))
i_slines = inspect.getsourcelines(type(i_loc))
i_slinf = inspect.getsourcelines(type(i_loc).isValid)

print("members")
print(i_members)
print(i_membmet)
print(i_membfun)
print("type of members")
print(type(i_members))
print("function")
print(i_funcg)
print(i_funcs)
print("sourcefile")
print(i_sfile)
print("file")
print(i_file)
print("source")
print(i_source)
print("sourcelines")
print(i_slines)
print(i_slinf)

i_crgba_msg = ColorRGBA()
i_rloc_msg = rosLocationMsg()
i_marker_msg = Marker

i_crgba_msg_inspec = inspect.getmembers(i_crgba_msg)
i_rloc_msg_inspec = inspect.getmembers(i_rloc_msg)
i_marker_msg_inspec = inspect.getmembers(i_marker_msg)

print("message inspection")
print("ColorRGBA")
print(i_crgba_msg_inspec)
for key, value in i_crgba_msg_inspec:
    if(key == "_fields_and_field_types"):
        print(value)
        print(type(value))
        for key, val in value.items():
            print(key)
            print(val)
print("rosLocation")
print(i_rloc_msg_inspec)
for key, value in i_rloc_msg_inspec:
    if(key == "_fields_and_field_types"):
        print(value)
print("Marker")
print(i_marker_msg_inspec)
for key, value in i_marker_msg_inspec:
    if(key == "_fields_and_field_types"):
        print(value)