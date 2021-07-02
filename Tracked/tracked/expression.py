'''
Created on 02.07.2021

@author: Enzo Brancaccio
'''

import math

import tracked.trackingHelpers as th

from numbers import Number

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
                n = n - th.is_operator(expr[x])
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
                    resultList.append(th.inverse_op(expr[i]))
                else:
                    resultList.append(th.inverse_op(expr[i]))
            else:
                if(is_rhs):
                    resultList.append(expr[i])
                else:
                    resultList.append(th.inverse_op(expr[i]))
            resultString = ";".join(resultList)
            
            return reverseExpression(expr, i + 1, (expr[i] == "swap")) + resultString   
    elif(isinstance(expr, str)):
        exprList = expr.split(";")
        return reverseExpression(exprList, 0)
    else:
        return ""