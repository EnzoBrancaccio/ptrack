'''
Created on 12.07.2021

@author: Enzo Brancaccio
'''

# Test: Tracked of list without needing Reference wrapper class
class RefDummy(object):
    '''
    classdocs
    '''


    def __init__(self, value):
        '''
        Constructor
        '''
        self.valueStore = None
        self.references = dict()
        
        # distinction may also be useful for Tracked
        '''
        if(isinstance(self.valueStore, list)):
            if(isinstance(value, list)):
                self.valueStore.extend(value)
            else: 
                self.valueStore.append(value)
        else:
            self.valueStore = value
        '''
        if(isinstance(value, list) and isinstance(self.valueStore, list)):
            self.valueStore.extend(value)
            print(type(self.valueStore))
        else:
            self.valueStore = value
            print(type(self.valueStore))
    
    '''        
    def __setattr__(self, attrName, value):
        super().__setattr__(attrName, value)
        if(isinstance(self, RefDummy)):
            if(isinstance(self.valueStore, list)):
                if(attrName in self.__dict__):
                    print("__setattr__ 1")
                    print(attrName)
                    print(value)
                    self.references[attrName] = value
                    print("__setattr__ 2")
                    for k, v in self.references.items():
                        print(k)
                        print(v)
                    print("__setattr__ 3")
                else:
                    raise AttributeError(f"Unknown attribute {attrName}")
    '''
            
    def __getitem__(self, position):
        print("getitem")
        if(isinstance(position, int)):
            return self.valueStore[position]
        else:
            raise TypeError("position must be int")
            
    def __setitem__(self, position, item):
        print("position")
        print(position)
        print("item")
        print(item)
        isTracked = isinstance(self, RefDummy)
        isTrackedList = isinstance(self.valueStore, list)
        if(isTracked and isTrackedList):
            self.valueStore[position] = item
            #self.__setattr__(self, position, item)