'''
Created on 01.06.2021

@author: Enzo Brancaccio
'''

import copy
import math
import src.ptracking.ptracking.locationMap as lm

from src.ptracking.ptracking.location import Location
from src.rosslt_msgs.msg import LocationHeader
from numbers import Number

class Tracked(object):
    '''
    classdocs
    '''
    
    def __init__(self, value, y = None):
        '''
        Constructor
        '''
        self.value = value
        self.location = Location()
        self.location_map = dict([(".", self.location)])
        self.rosslt_msg = None
        
        if(not (y is None)):
            if(isinstance(y, Location)):
                self.location = y
                self.location_map = dict([(".", self.location)])
            if(isinstance(y, dict)):
                self.locationMap = y
            if(isinstance(y, LocationHeader)):
                for i in range(len(y.paths)):
                    self.location_map[y.paths[i]] = y.locations[i]
    
    # overloading the dot . (attribute access)
    # called when attr is not one of object's writable attributes
    def __getattr__(self, attr):
        """Override attribute access
        
        Keyword arguments:
        self -- Tracked object
        attr -- Name of attribute not in Tracked's attribute list
        
        Allows direct access to ROS message attributes if it's the Tracked.value
        For example, instead of Tracked.value.source_node, write Tracked.source_node
        Also possible is Tracked.source_node = "some value"
        Replaces GET_FIELD and SET_FIELD
        A Tracked object is returned, e. g. to access the value write Tracked.souce_node.value 
        """
        try:
            attr_value = ""
            tracked_attr = ""
            super().__setattr__(attr_value, getattr(self.value, attr))
            # returns a Tracked object instead of the value directly
            super().__setattr__(tracked_attr, Tracked(attr_value, lm.locationSlice(self.location_map, attr)))
            return tracked_attr
        except:
            raise AttributeError(attr)

    def __setattr__(self, attr, value):
        """Override attribute access
        
        Keyword arguments:
        self  -- Tracked object
        attr  -- Name of attribute
        value -- New value for attribute
        
        Implementing the SET_FIELD functionality
        If value is a Tracked object, self's location_map is updated 
        """
        if(hasattr(self, "location_map")):
            if(isinstance(value, Tracked)):
                super().__setattr__("self.location_map", lm.addLocations(self.location_map, value.location_map, attr))
        super().__setattr__(attr, value)
    
    def __add__(self, other):
        """Override the + operator
        
        Keyword arguments:
        self -- Tracked object
        other -- Right-hand object added to self
        
        x + y => x.__add__(y)
        Here: self + other = self.__add__(other)
        Allows adding to a left-hand side Tracked object's value
        No need to write Tracked.value explicitly, Tracked alone also works
        Covers the cases that "other" is Tracked or not
        Covers string concatenation
        """
        isSelfTracked = isinstance(self, Tracked)
        isOtherTracked = isinstance(other, Tracked)
        if(isSelfTracked and (not isOtherTracked)):
            copiedVar = copy.deepcopy(self)
            copiedVar.value = self.value + other
            expr = copiedVar.location_map["."].expression
            copiedVar.location_map["."].expression = expr + str(other) + ";+;"
            return copiedVar
        elif(isSelfTracked and isOtherTracked):
            copiedVar = copy.deepcopy(self)
            copiedVar.value = self.value + other.value
            expr = copiedVar.location_map["."].expression
            copiedVar.location_map["."].expression = expr + str(other.value) + ";+;"
            return copiedVar
        else:
            newValue = self + other
            return newValue
        
    def __radd__(self, other):
        """Special case of overriding the + operator
        
        Keyword arguments:
        self -- Tracked object
        other -- Right-hand object added to self
        
        x + y => y.__radd__(x)
        Here: other + self = self.__add__(other)
        Special case of left-hand side object not being of type Tracked
        Allows adding to a right-hand side Tracked object's value
        No need to write Tracked.value explicitly, Tracked alone also works
        Covers string concatenation
        """
        isSelfTracked = isinstance(self, Tracked)
        if(isSelfTracked):
            copiedVar = copy.deepcopy(self)
            copiedVar.value = other + self.value # str concatenation
            expr = copiedVar.location_map["."].expression
            copiedVar.location_map["."].expression = expr + str(other) + ";swap;+;"
            return copiedVar
        
    def __sub__(self, other):
        """Override the - operator
        
        Keyword arguments:
        self -- Tracked object
        other -- Right-hand object subtracted from self
        
        x - y => x.__sub__(y)
        Here: self - other = self.__sub__(other)
        Allows subtracting from a left-hand side Tracked object's value
        No need to write Tracked.value explicitly, Tracked alone also works
        Covers the cases that "other" is Tracked or not
        """
        isSelfTracked = isinstance(self, Tracked)
        isOtherTracked = isinstance(other, Tracked)
        if(isSelfTracked and (not isOtherTracked)):
            copiedVar = copy.deepcopy(self)
            copiedVar.value = self.value - other
            expr = copiedVar.location_map["."].expression
            copiedVar.location_map["."].expression = expr + str(other) + ";-;"
            return copiedVar
        elif(isSelfTracked and isOtherTracked):
            copiedVar = copy.deepcopy(self)
            copiedVar.value = self.value - other.value
            expr = copiedVar.location_map["."].expression
            copiedVar.location_map["."].expression = expr + str(other.value) + ";-;"
            return copiedVar
        else:
            newValue = self - other
            return newValue
          
    def __rsub__(self, other):
        """Special case of overriding the - operator
        
        Keyword arguments:
        self -- Tracked object
        other -- Right-hand object subtracting self
        
        x - y => y.__rsub__(x)
        Here: other - self = self.__rsub__(other)
        Special case of left-hand side object not being of type Tracked
        Allows subtracting from a right-hand side Tracked object's value
        No need to write Tracked.value explicitly, Tracked alone also works
        """
        isSelfTracked = isinstance(self, Tracked)
        if(isSelfTracked):
            copiedVar = copy.deepcopy(self)
            copiedVar.value = other - self.value
            expr = copiedVar.location_map["."].expression
            copiedVar.location_map["."].expression = expr + str(other) + ";swap;-;"
            return copiedVar
       
    def __mul__(self, other):
        """Override the * operator
        
        Keyword arguments:
        self -- Tracked object
        other -- Right-hand object multiplied with self
        
        x * y => x.__mul__(y)
        Here: self * other = self.__mul__(other)
        Allows multiplying with a left-hand side Tracked object's value
        No need to write Tracked.value explicitly, Tracked alone also works
        Covers the cases that "other" is Tracked or not
        Covers the case of other being 0
        """
        isSelfTracked = isinstance(self, Tracked)
        isOtherTracked = isinstance(other, Tracked)
        if(isSelfTracked and isOtherTracked):
            copiedVar = copy.deepcopy(self)
            copiedVar.value = self.value * other.value
            if(self.location_map["."].isValid()):
                isOtherNull = other.value == 0
                isOtherNumber = isinstance(other.value, Number)
                if(isOtherNull and isOtherNumber):
                    copiedVar.location_map["."] = other.location
                    expr = copiedVar.location_map["."].expression
                    copiedVar.location_map["."].expression = expr + str(self) + ";swap;*;"
                    return copiedVar
                copiedVar.location_map["."] = self.location
                expr = copiedVar.location_map["."].expression
                copiedVar.location_map["."].expression = expr + str(other.value) + ";*;"
                return copiedVar
        elif(isSelfTracked):
            copiedVar = copy.deepcopy(self)
            copiedVar.value = self.value * other
            expr = copiedVar.location_map["."].expression
            copiedVar.location_map["."].expression = expr + str(other) + ";*;"
            isOtherNull = other == 0
            isOtherNumber = isinstance(other, Number)
            if(isOtherNull and isOtherNumber):
                copiedVar.location_map["."] = Location()
            return copiedVar
        else:
            newValue = self * other
            return newValue
      
    def __rmul__(self, other):
        """Special case of overriding the * operator
        
        Keyword arguments:
        self -- Tracked object
        other -- Right-hand object multiplied with self
        
        x * y => y.__rmul__(x)
        Here: other * self = self.__rmul__(other)
        Special case of left-hand side object not being of type Tracked
        Allows multiplying with a right-hand side Tracked object's value
        No need to write Tracked.value explicitly, Tracked alone also works
        Covers the case of other being 0
        """
        isSelfTracked = isinstance(self, Tracked)
        if(isSelfTracked):
            copiedVar = copy.deepcopy(self)
            copiedVar.value = other * self.value
            expr = copiedVar.location_map["."].expression
            copiedVar.location_map["."].expression = expr + str(other) + ";swap;*;"
            isOtherNull = other == 0
            isOtherNumber = isinstance(other, Number)
            if(isOtherNumber and isOtherNull):
                copiedVar.location_map["."] = Location()
            return copiedVar
    
    def __truediv__(self, other):
        """Override the / operator
        
        Keyword arguments:
        self -- Tracked object
        other -- Right-hand object self is divided by
        
        x / y => x.__truediv__(y)
        Here: self / other = self.__truediv__(other)
        Allows dividing a left-hand side Tracked object's value
        No need to write Tracked.value explicitly, Tracked alone also works
        Covers the cases that "other" is Tracked or not
        """
        isSelfTracked = isinstance(self, Tracked)
        isOtherTracked = isinstance(other, Tracked)
        if(isSelfTracked and isOtherTracked):
            copiedVar = copy.deepcopy(self)
            copiedVar.value = self.value / other.value
            expr = copiedVar.location_map["."].expression
            copiedVar.location_map["."].expression = expr + str(other.value) + ";/;"
            return copiedVar
        elif(isSelfTracked and (not isOtherTracked)):
            copiedVar = copy.deepcopy(self)
            copiedVar.value = self.value / other
            expr = copiedVar.location_map["."].expression
            copiedVar.location_map["."].expression = expr + str(other) + ";/;"
            return copiedVar
        else:
            newValue = self / other
            return newValue
     
    def __rtruediv__(self, other):
        """Special case of overriding the / operator
        
        Keyword arguments:
        self -- Tracked object
        other -- Right-hand object divided by self
        
        x / y => y.__rtruediv__(x)
        Here: other / self = self.__rtruediv__(other)
        Special case of left-hand side object not being of type Tracked
        Allows dividing by a right-hand side Tracked object's value
        No need to write Tracked.value explicitly, Tracked alone also works
        """
        isSelfTracked = isinstance(self, Tracked)
        if(isSelfTracked):
            copiedVar = copy.deepcopy(self)
            copiedVar.value = other / self.value
            expr = copiedVar.location_map["."].expression
            copiedVar.location_map["."].expression = expr + str(other) + ";swap;/;"
            return copiedVar
        
    def sin(self):
        """Sine function for Tracked based on math.sin
        
        Keyword arguments:
        self -- Tracked object
        
        Returns the sine of Tracked.value if it's a number
        """
        isSelfTracked = isinstance(self, Tracked)
        isValueNumber = isinstance(self.value, Number)
        if(isSelfTracked and isValueNumber):
            copiedVar = copy.deepcopy(self)
            copiedVar.value = math.sin(self.value)
            expr = copiedVar.location_map["."].expression
            copiedVar.location_map["."].expression = expr + "sin;"
            return copiedVar
        
    def cos(self):
        """Cosine function for Tracked based on math.cos
        
        Keyword arguments:
        self -- Tracked object
        
        Returns the cosine of Tracked.value if it's a number
        """
        isSelfTracked = isinstance(self, Tracked)
        isValueNumber = isinstance(self.value, Number)
        if(isSelfTracked and isValueNumber):
            copiedVar = copy.deepcopy(self)
            copiedVar.value = math.cos(self.value)
            expr = copiedVar.location_map["."].expression
            copiedVar.location_map["."].expression = expr + "cos;"
            return copiedVar
    
    def size(self):
        """Length of Tracked.value if it's a list
        
        Keyword arguments:
        self -- Tracked object
        
        Uses list for Tracked.value (vector in C++ Tracked)
        """
        if(isinstance(self, Tracked)):
            if(isinstance(self.value, list)):
                return len(self.value)
            
    def append(self, inputArg):
        """Override the append method covering Tracked objects
        
        Keyword arguments:
        self -- Tracked object
        inputArg -- Object appended to Tracked.value == list
        
        Tracked.value is a list of Tracked and object shall be added to its end
        If inputArg is not Tracked then it's turned into a Tracked object
        Necessity to avoid using append itself because of __setitem__ overriding
        """
        if(isinstance(self, Tracked)):
            if(isinstance(self.value, list)):
                if(isinstance(inputArg, Tracked)):
                    # for overloading of __setitem__ to take effect
                    self[len(self.value)] = inputArg
                    self.location_map = lm.addLocations(self.location_map, inputArg.location_map, str(self.size() - 1))
                else:
                    self[len(self.value)] = self.make_tracked(inputArg)
                    
    def pop_back(self):
        """Remove element from end of Tracked.value list
        
        Keyword arguments:
        self -- Tracked object
        
        Equivalent of C++ std::vector<T,Allocator>::pop_back
        """
        if(isinstance(self, Tracked)):
            if(isinstance(self.value, list)):
                del(self.value[-1])
                self.location_map = lm.removeLocations(self.location_map, str(self.size()))
                
    def clear(self):
        """Clears the Tracked.value list
        
        Keyword arguments:
        self -- Tracked object
        
        Uses Python's own clear() methods for lists
        """
        if(isinstance(self, Tracked)):
            if(isinstance(self.value, list)):
                self.value.clear()
                self.location_map = lm.locationSlice(self.location_map, ".")
                    
    def front(self):
        """Return the first element of Tracked.value list
        
        Keyword arguments:
        self -- Tracked object
        
        The list itself remains unchanged
        """
        if(isinstance(self, Tracked)):
            if(isinstance(self.value, list)):
                return self.value[0]
            
    def back(self):
        """Return the last element of Tracked.value list
        
        Keyword arguments:
        self -- Tracked object
        
        In a Python list, -1 is the index of the last element
        """
        if(isinstance(self, Tracked)):
            if(isinstance(self.value, list)):
                return self.value[-1]
    
    def __getitem__(self, position):
        """Override the [] operator for retrieving from Tracked.value list
        
        Keyword arguments:
        self -- Tracked object
        position -- Index of element in Tracked.value list
        
        Directly access elements in Tracked.value list 
        For example, write Tracked[position] instead of Tracked.value[position]
        Covers the case that the index does not exist in the list
        Covers the case of a negative index for it to behave like in normal Python lists
        """
        isTracked = isinstance(self, Tracked)
        isTrackedList = isinstance(self.value, list)
        if(isTracked and isTrackedList):
            listLength = len(self.value)
            if(position >= listLength):
                raise IndexError(f"Index is {position} and list length is {listLength}")
            trackedValue = self.value[position]
            # negative index: -1 for last list element etc.
            if(position < 0):
                actualPosition = list(self.location_map.keys())[position]
                if(self.location_map[actualPosition].isValid()):    
                    trackedLocation = self.location_map[actualPosition]
                    return self.make_tracked(trackedValue, trackedLocation)
                else:
                    return self.make_tracked(trackedValue)
            else:
                if(self.location_map[str(position)].isValid()):    
                    trackedLocation = self.location_map[str(position)]
                    return self.make_tracked(trackedValue, trackedLocation)
                else:
                    return self.make_tracked(trackedValue)
        else:
            try:
                return self[position]
            except IndexError:
                raise IndexError(f"Index is {position} and list length is {len(self)}")
          
    def __setitem__(self, position, item):
        """Override the [] operator for adding to Tracked.value list
        
        Keyword arguments:
        self -- Tracked object
        position -- Index of element in Tracked.value list
        item -- Element to be added to Tracked.value list at index position
        
        Directly add elements to Tracked.value list at index
        For example, write Tracked[position] = item instead of Tracked.value[position] = item
        Covers the cases that item is Tracked or not
        If the position is an invalid index, the element is appended
        """
        isTracked = isinstance(self, Tracked)
        isTrackedList = isinstance(self.value, list)
        if(isTracked and isTrackedList):
            isItemTracked = isinstance(item, Tracked)
            if(isItemTracked):
                if(len(self.value) < position):
                    self.value[position] = item.value
                else:
                    self.value.append(item.value)
                self.location_map = lm.addLocations(self.location_map, item.location_map, str(position))
            else:
                if(len(self.value) < position):
                    self.value[position] = item
                else:
                    self.value.append(item)
                self.location_map = lm.addLocations(self.location_map, dict(), "")

    def get_field(self, attr):
        """Return field as Tracked object
        
        Keyword arguments:
        self -- Outer Tracked object
        attr -- Name of attribute to get value from
        
        Returns the value as Tracked object if Tracked.value is not a Tracked object itself.
        E. g. Tracked.value is a SourceChange message, so Tracked.source_node is a string.
        In such a case, overriding __getattr__, as was done above, is not enough,
        because it just overrides the access for Tracked objects, and not globally.
        When it's known that Tracked.value is also Tracked, this method is not necessary,
        else, like in our example, Tracked.get_field("source_node") returns a Tracked object.
        """
        attr_value = getattr(self, attr)
        if(not isinstance(attr_value, Tracked)):
            return Tracked(attr_value, lm.locationSlice(self.location_map, attr))
        else:
            attr_value.location_map = lm.locationSlice(self.location_map, attr)
            return attr_value

    def make_tracked(self, value, location = None):
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