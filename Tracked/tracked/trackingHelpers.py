'''
Created on 24.06.2021

@author: Enzo Brancaccio
'''

import math

from .location import Location
from .tracked import Tracked
from numbers import Number

def makeTracked(value, location = None):
    if(isinstance(value, Tracked)):
        return value
    else:
        if(location is None):
            return Tracked(value, Location())
        else:
            return Tracked(value, location)

# from helper.h
def applyExpression(value, expression):
    if(isinstance(value, Number)):
        stack = list()
        stack.append(value)
        # using split to turn expression into list
        exprList = expression.split(";")
        for token in exprList:
            if(token == ""):
                continue
            elif(token == "+"):
                stack[-2] = stack[-2] + stack[-1]
                stack.pop()
            elif(token == "-"):
                stack[-2] = stack[-2] - stack[-1]
                stack.pop()
            elif(token == "*"):
                stack[-2] = stack[-2] * stack[-1]
                stack.pop()
            elif(token == "/"):
                stack[-2] = stack[-2] / stack[-1]
                stack.pop()
            elif(token == "swap"):
                stack[-2], stack[-1] = stack[-1], stack[-2]
            elif(token == "sin"):
                stack[-1] = math.sin(stack[-1])
            elif(token == "cos"):
                stack[-1] = math.cos(stack[-1])
            elif(token == "asin"):
                stack[-1] = math.asin(stack[-1])
            elif(token == "acos"):
                stack[-1] = math.acos(stack[-1])
            elif(token[0] == '"'):
                if(token[-1] == '"'):
                    # result of split is like ['', x, ''] so we take index 1
                    newValue = float(token.split('\"')[1])
                    stack.append(newValue)
                else:
                    pass # not yet sure which case is covered
            else:
                stack.append(float(token))
        return stack[-1]

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