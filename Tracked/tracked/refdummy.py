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
        else:
            self.valueStore = value
            
    def __setattr__(self, attrName, value):
        super().__setattr__(attrName, value)
        if(isinstance(self, RefDummy)):
            if(isinstance(self.valueStore, list)):
                if(attrName in self.__dict__):
                    print(attrName)
                    print(value)
                    self.references[attrName] = value
                    for k, v in self.references.items():
                        print(k)
                        print(v)
                else:
                    raise AttributeError(f"Unknown attribute {attrName}")
                
        