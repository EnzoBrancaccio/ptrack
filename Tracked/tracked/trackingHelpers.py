'''
Created on 24.06.2021

@author: Enzo Brancaccio
'''

from .location import Location
from .tracked import Tracked
from numbers import Number

def make_tracked(value, location = None):
    if(isinstance(value, Tracked)):
        return value
    else:
        if(location is None):
            return Tracked(value, Location())
        else:
            return Tracked(value, location)

# interprets attributes unknown to object
# every case needs to be covered    
def interpret_dot_attr(obj, name):
    if("foo" in name):
        print(name)
        return obj.value
    elif("bar" in name):
        print(name)
        obj.value = 7
    elif("setTo" in name):
        # note that just "set" is not possible because of setstate etc.
        print(name)
        splitName = name.split("_")
        newValue = splitName[-1]
        print(newValue)
        obj.value = newValue
    else:
        raise AttributeError(name)

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
    exprList = list()
    exprTempList = string.split(";")
    # string ending with ";" results in "" being appended
    for x in exprTempList:
        if(x != ""):
            exprList.append(x)
    return exprList

def createExpressionString(exprList):
    expressionString = ";".join(exprList)
    expressionString = ";" + expressionString + ";"
    return expressionString