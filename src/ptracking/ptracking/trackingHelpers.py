'''
Created on 24.06.2021

@author: Enzo Brancaccio
'''

import inspect
from rclpy.executors import Executor
from rclpy.task import Future
from src.ptracking.ptracking.location import Location
from src.ptracking.ptracking.tracked import Tracked
from numbers import Number
from rclpy.node import Node

def make_tracked(value, location = None):
    """Create a new Tracked object
        
    Keyword arguments:
    value -- New Tracked.value
    location -- Location of new Tracked if provided
        
    New Tracked object is created with value and location as paramaters
    """
    if(isinstance(value, Tracked)):
        return value
    else:
        if(location is None):
            return Tracked(value, Location())
        else:
            return Tracked(value, location)

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
        
        