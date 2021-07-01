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
                    # case: expression inside expression, split over several tokens
                    subList = list()
                    firstIndex = 0
                    lastIndex = 0
                    for subToken in exprList:
                        if(subToken != ""):
                            if(subToken[0] == '"'):
                                firstIndex = exprList.index(subToken)
                            if(subToken[-1] == '"'):
                                lastIndex = exprList.index(subToken)
                    # strings beginning with ", ending with " and those in between
                    subList = exprList[firstIndex:(lastIndex+1)]
                    # remove the subList-strings from exprList so they don't interfere
                    del exprList[firstIndex:(lastIndex+1)]
                    # turn list into string, separated by ;                      
                    newValue = ";".join(subList)
                    # remove quotation marks from string (they are elements with index 0 and 2 in list)
                    newValue = newValue.split('"')[1]
                    stack.append(newValue) 
            else:
                stack.append(float(token))
        return stack[-1]
    elif(isinstance(value, str)):
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
                minusLength = len(stack[-2]) - len(stack[-1])
                stack[-2] = stack[-2][0:minusLength]
                stack.pop()
            elif(token == "swap"):
                stack[-2], stack[-1] = stack[-1], stack[-2]
            elif(token[0] == '"'):
                if(token[-1] == '"'):
                    # result of split is like ['', x, ''] so we take index 1
                    newValue = token.split('\"')[1]
                    stack.append(newValue)
                else:
                    # case: expression inside expression, split over several tokens
                    subList = list()
                    firstIndex = 0
                    lastIndex = 0
                    for subToken in exprList:
                        if(subToken != ""):
                            if(subToken[0] == '"'):
                                firstIndex = exprList.index(subToken)
                            if(subToken[-1] == '"'):
                                lastIndex = exprList.index(subToken)
                    # strings beginning with ", ending with " and those in between
                    subList = exprList[firstIndex:(lastIndex+1)]
                    # remove the subList-strings from exprList so they don't interfere
                    del exprList[firstIndex:(lastIndex+1)]
                    # turn list into string, separated by ;                      
                    newValue = ";".join(subList)
                    # remove quotation marks from string (they are elements with index 0 and 2 in list)
                    newValue = newValue.split('"')[1]
                    stack.append(newValue)                     
            else:
                stack.append(token)
        return stack[-1]

# from trackingnode.cpp    
def reverseExpression(expr, position = None, swapped = False):
    if(isinstance(expr, list)):
        if(position >= len(expr)):
            return ""
        else:
            resultList = list()
            
            n = 0
            i = position
            for x in range(position, len(expr)):
                n = n - is_operator(expr[x])
                if(n < 0):
                    break
                else:
                    n = n + 1
                    i = i + 1
                    
            for y in range(position, i):
                resultList.append(expr[y])
            
            is_rhs = swapped or (((i - 1) >= 0) and (expr[i-1] == "swap"))
            is_commutative = (expr[i] == "+") or (expr[i] == "*")
            if(is_commutative):
                if(is_rhs):
                    resultList.append("swap")
                    resultList.append(inverse_op(expr[i]))
                else:
                    resultList.append(inverse_op(expr[i]))
            else:
                if(is_rhs):
                    resultList.append(expr[i])
                else:
                    resultList.append(inverse_op(expr[i]))
            resultString = ";".join(resultList)
            
            return reverseExpression(expr, i + 1, (expr[i] == "swap")) + resultString   
    elif(isinstance(expr, str)):
        exprList = expr.split(";")
        return reverseExpression(exprList, 0)
    else:
        return ""

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