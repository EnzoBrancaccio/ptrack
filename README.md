# rosslt (Python)

This project is the Python implementation of the C++ [rosslt](https://gitlab.uni-ulm.de/s_twitt1/quadcopter-doku/-/blob/master/source/includes/_werkzeuge.md#rosslt) library, as was suggested in T. Witte and M. Tichy, "Inferred Interactive Controls Through Provenance Tracking of ROS Message Data," *2021 IEEE/ACM 3rd International Workshop on Robotics Software Engineering (RoSE)*, 2021, pp. 67-74, doi: 10.1109/RoSE52553.2021.00018.

## Tracked Wrapper

The wrapper is a Tracked object taking two parameters:
- `value` (the wrapped object, e. g. a number or string etc.)
- `location` (optional)
It has additional attributes `location_map` (no own class, just a dictionary) and `rosslt_msg`.
To make coding with a Tracked object easier and reduce the amount of extra code due to the wrapping, several operators were overridden:

### Arithmetic operations: +, -, *, /, sin, cos
Allows calculating with a Tracked object as if there was no wrapping involved. An example from the tests:
``` Python
self.a = Tracked(5.0) # a is new Tracked object with value 5.0
self.b = self.a + 2.0 # b is a new Tracked object
# self.b.value = 7.0
```
As can be seen above, you can calculate with Tracked objects like with normal variables, but also note that to check the value, access the value attribute.

### Attribute access: . and []
To avoid the large amount of extra code due to the `SET_FIELD` and `GET_FIELD` methods in C++, Tracked's dot operator (`__getattr__`) was overwritten to grant direct access the attributes of Tracked ROS message. An example:
``` Python
self.tmsg = Tracked(SourceChange) # Message type with fields source_node, location_id and new_value
self.tmsg.source_node = "foo"     # Not necessary: self.tsmg.value.source_node = "foo"
self.name = self.tsmg.source_node # Not necessary: self.name = self.tsmg.value.source_node
# self.name = "foo"
```
The brackets operator (`__getitem__`, `__setitem__`) deals with the case that the Tracked value is a list. Accessing a list element becaomes easier by writing, for example, `Tracked[index]` instead of `Tracked.value[index]`. Further methods have been overridden, like `size` or `append`, or were added like `pop_back` (same name as a C++ method, removing the last element of a list), `clear` (empty the list), and `front` and `back` (returning the first or last element of a list respectively).