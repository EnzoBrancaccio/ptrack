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
If Tracked.value is itself a Tracked object, it works without the need for an extra `GET_FIELD` method, but the value has to be accessed as value like for other Tracked objects, e. g. `Tracked.value` is a Tracked object, so access the value with `Tracked.value.value`.
An extra `get_field` method was added for the case when Tracked.value is not a Tracked object also. In the above example, `self.tmsg.source_node` is a string and calls its own `__getattr__` method, not the overridden Tracked version, making a new method necessary.
The `__setattr__` method was also overridden to update the Tracked object's location_map (in case the new value is a Tracked object).

The brackets operator (`__getitem__`, `__setitem__`) deals with the case that the Tracked value is a list. Accessing a list element becomes easier by writing, for example, `Tracked[index]` instead of `Tracked.value[index]`. Further methods have been overridden, like `size` or `append`, or were added like `pop_back` (same name as a C++ method, removing the last element of a list), `clear` (empty the list), and `front` and `back` (returning the first or last element of a list respectively).

### ROSSLT messages
Two methods allow the easy conversion of Tracked objects to messages and tracked messages to Tracked objects, mostly for dealing with tracked versions (`Int32Tracked`, `PoseTracked` and `MarkerTracked`) of standard messages (`Int32`, `Pose` and `Marker`):
- `toTrackedMsg` to convert Tracked objects whose `value` is a standard message into the tracked version of the message. The parameter it receives is the tracked message type.
- `incorporateTrackedMsg` to convert the tracked version of a message into a Tracked object whose `value` is the equivalent standard message. This implementation was chosen because `Pose` and `Marker` messages have more and differing attributes than `Int32` (which has just a `data` field), so turning them all into a Tracked object whose `value` contains the `data` and that is still easy to use is not possible. For example, when `Int32Tracked` is turned into a Tracked object, the data field can be accessed by writing `Tracked.value.data`.

In addition, `trackingHelpers.py` contains two methods, `createTrackedFromTrackedMsg` and `createTrackedMsgFromTracked`, that do the same without the need of having (or having to create) a Tracked object that provides the methods first.   

## Architecture

The project has been split up into several modules, one for each class or related functions. Examles for the latter are a module `expression.py` with the `applyExpression` and `reverseExpression` functions, `locationMap.py` for the location map functions (since location map is just a dictionary here), and `trackingHelpers.py` with independent functions for the Tracked and TrackingNode classes. An example for the former is the class `LocationManager` having its own module and no longer being together with `TrackingNode`.

### Constructors
In the C++ version, `Tracked` and `Location` have multiple constructors, e. g. a `Location` (in `location.h`) can be initialised with `location_id` and `source_node`, or with a ROS location message. To avoid extra code (e g. with `@classmethod` based solutions), the `Tracked` and `Location` classes have at least one unspecified parameter that defaults to `None` and is the value is checked in the body of the `__init__` method via if-statements. This is not more code than adding classmethods and allows the user to create new instances of those classes without extra code (e. g. something like `Location.rosLocationMsg(...)`).

### LocationFunc
Has its own class (`struct` in `trackingNode.h`), takes no arguments but `self`, has two attributes, `self.location_id` and ``self.new_value` and a `get` and `set` method. In C++, it roughly works like an interface (see lines 142-152 in `trackingNode.h`). A similar behavior is achieved in `trackingNode.py`'s `loc`-method, by defining two nested methods `_get` and `_set` and then use them to override a `LocationFunc` instance's `get` and `set` method via the `types` library, using the `MethodType` function.

### LocationManager
The datatype of `self.source_locations` is a dictionary with key `source_location` (a string) and value `int`, while `self.locations` is a list of instances of `LocationFunc` objects. The dict `self.source_locations` is filled in method `create_location` in case the provided `source_location` is not yet in the dict.

### TrackingNode
The reevaluate and map_leaves methods of C++ are all in this class and have been turned into the methods
- `reevaluate`: main method and checks if value is of type message or not (if yes, it calls `reevaluate_msg`)
- `reevaluate_msg`: extracts the fields and calls `reevaluate_generic`
- `reevaluate_submsg`: for recursive calls when a message contains a message etc.
- `reevaluate_generic`: checks if a message contains a message and then calls `reevaluate_submsg` again
- `reevaluate_value`: does the actual reevaluating and gets called when the object is not nested
![Connection between reevaluate methods](/reevaluate_diag.svg)
The relationship between the methods is shown in the above diagram. With `reevaluate_msg` and `reevaluate_generic`, the methods go deeper into nested message structures.