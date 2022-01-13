'''
Created on 24.06.2021

@author: Enzo Brancaccio
'''

import inspect
from rclpy.executors import Executor
from numbers import Number
from src.ptracking.ptracking.tracked import Tracked
from src.ptracking.ptracking.location import Location
from src.rosslt_msgs.msg import LocationHeader
from src.rosslt_msgs.msg import Int32Tracked
from src.rosslt_msgs.msg import MarkerTracked
from src.rosslt_msgs.msg import PoseTracked
from std_msgs.msg import Int32
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose

def inv_plus(lhs, rhs):
    isNumeric = ((isinstance(lhs, Number)) and (isinstance(rhs, Number)))
    if(isNumeric):
        return lhs - rhs

def inv_minus(lhs, rhs):
    isNumeric = ((isinstance(lhs, Number)) and (isinstance(rhs, Number)))
    if(isNumeric):
        return lhs + rhs

def inv_mul(lhs, rhs):
    isNumeric = ((isinstance(lhs, Number)) and (isinstance(rhs, Number)))
    if(isNumeric):
        return lhs / rhs

def inv_div(lhs, rhs):
    isNumeric = ((isinstance(lhs, Number)) and (isinstance(rhs, Number)))
    if(isNumeric):
        return lhs * rhs
    
def inverse_op(operator):
    if(operator == "+"):
        return "-"
    elif(operator == "-"):
        return "+"
    elif(operator == "*"):
        return "/"
    elif(operator == "/"):
        return "*"
    elif(operator == "swap"):
        return "swap"
    elif(operator == "sin"):
        return "asin"
    elif(operator == "cos"): 
        return "acos"
    elif(operator == "asin"): 
        return "sin"
    elif(operator == "acos"): 
        return "cos"
    else:
        raise ValueError("no valid operator '" + operator + "'")

def is_operator(token):
    if(token == "+"): 
        return 2
    elif(token == "-"): 
        return 2;
    elif(token == "*"): 
        return 2;
    elif(token == "/"): 
        return 2;
    elif(token == "sin"): 
        return 1;
    elif(token == "cos"): 
        return 1;
    elif(token == "asin"): 
        return 1;
    elif(token == "acos"): 
        return 1;
    elif(token == "swap"): 
        return 1;
    else:
        return 0
    
def createExpressionList(string):
    """Creates a list of expression strings
        
    Keyword arguments:
    string -- String of semicolon-separated expressions
        
    Takes the expression string and splits it with semicolon as delimiter
    Returns list of single expression strings
    """
    exprList = list()
    exprTempList = string.split(";")
    # string ending with ";" results in "" being appended
    for x in exprTempList:
        if(x != ""):
            exprList.append(x)
    return exprList

def createExpressionString(exprList):
    """Creates a string of expressions
        
    Keyword arguments:
    exprList -- List of expression strings
        
    Joins the expression strings to one String using as delimiter semicolon
    Returns string of expressions
    """
    expressionString = ";".join(exprList)
    expressionString = ";" + expressionString + ";"
    return expressionString

def get_future(node, future):
    # method always returns None
    Executor.spin_until_future_complete(node, future)
    # from rclpy.task.Future
    isCompletedSuccessfully = (future.done() and (not future.cancelled()))
    if(isCompletedSuccessfully):
        return future
    else:
        node.get_logger().info("Error getting result from future")

def extract_fields(obj, attr_key):
    """Return the fields of an object
        
    Keyword arguments:
    obj -- Object to be inspected
    attr_key -- Key to look for in list from inspection
        
    Inspects object (a message) to get a list of the attributes
    Go through the list to find the dictionary with provided key
    Return the dictionary (key: fieldname, value: fieldtype) 
    """
    # using inspect.getmembers() because messages use slots
    # attrs is a list
    attrs = inspect.getmembers(obj)
    fields = dict()
    for key, value in attrs:
        if(key == attr_key):
            fields = value
    return fields

def create_source_location_str(frame, key):
    """Return string to use as source location key
        
    Keyword arguments:
    frame -- List of frame records
    key -- Key of the frame with method call
        
    Takes frame and key (usually 1, since 0 is the frame in which getcurrentframe() was called)
    Extracts line number and code context, i. e. when TrackingNode's loc-function was called
    Turns them into a string to use as key for the source location number
    Returns the string 
    """
    o_frames = inspect.getouterframes(frame)
    sl_frame = o_frames.pop(key)
    sl_lineno = sl_frame.lineno
    # list with 1 element
    sl_cc = sl_frame.code_context.pop(0)
    # strip it off leading whitespaces
    sl_cc = sl_cc.lstrip()
    source_location = f"{sl_lineno} {sl_cc}"
    return source_location
        
def createTrackedFromTrackedMsg(tracked_msg):
    """Turn tracked version of standard message into Tracked object
        
    Keyword arguments:
    tracked_msg -- tracked version of standard message (Int32Tracked, PoseTracked, MarkerTracked)
        
    3 ROSSLT messages are tracked versions of standard messages, Int32Tracked, PoseTracked, MarkerTracked, 
    and have the same attributes:
    - data: untracked version of message (Int32, Pose or Marker)
    - location: LocationHeader message
    The tracked message is turned into a Tracked object such that
    - Tracked.value = untracked version of message (Int32, Pose or Tracked)
    - Tracked.location_map = LocationHeader message converted to a location_map
    Returns the Tracked object.
    This method is closely related to Tracked's incorporateTrackedMsg and lh_to_lm methods
    and allows the creation of a new Tracked object from the message. 
    """
    temp_lm = dict()
    for i in range(len(tracked_msg.location.paths)):
        new_location = Location(tracked_msg.location.locations[i])
        new_string = tracked_msg.location.paths[i]
        temp_lm[new_string] = new_location

    tracked = Tracked(tracked_msg.data, temp_lm)

    return tracked

def createTrackedMsgFromTracked(tracked):
    """Turn Tracked standard message into Tracked version of message
        
    Keyword arguments:
    tracked -- Tracked object whose value is a standard message (Int32, Pose, Marker) with tracked message version
        
    Tracked object whose value is one of the standard messages (Int32, Pose, Tracked)
    that has a tracked message version (Int32Tracked, PoseTracked, MarkerTracked)
    can be turned into the respective tracked message:
    - Tracked.value = Int32  -> Int32Tracked
    - Tracked.value = Pose   -> PoseTracked
    - Tracked.value = Marker -> MarkerTracked
    Returns the Tracked version of the message.
    This method is closely related to Tracked's toTrackedMsg and createLocationHeader methods
    and allows the creation of a new tracked message from a Tracked object. 
    """
    standardMsgs = (type(Int32()), type(Marker()), type(Pose()))
    if(isinstance(tracked.value, standardMsgs)):
        lh = LocationHeader()
        paths = list()
        locations = list()

        for key, value in tracked.location_map.items():
            paths.append(key)
            loc_msg = value.makeRossltLocationMsg()
            locations.append(loc_msg)

        lh.paths = paths
        lh.locations = locations

        if(type(tracked.value), type(Int32())):
            newInt32Tracked = Int32Tracked()
            newInt32Tracked.data = tracked.value
            newInt32Tracked.location = lh

            return newInt32Tracked

        if(type(tracked.value), type(Marker())):
            newMarkerTracked = MarkerTracked()
            newMarkerTracked.data = tracked.value
            newMarkerTracked.location = lh

            return newMarkerTracked

        if(type(tracked.value), type(Pose())):
            newPoseTracked = PoseTracked()
            newPoseTracked.data = tracked.value
            newPoseTracked.location = lh

            return newPoseTracked
    else:
        # assuming that Tracked.value is a message that needs no further processing
        return tracked.value