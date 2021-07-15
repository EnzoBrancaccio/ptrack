'''
Created on 14.07.2021

@author: Enzo Brancaccio
'''

class Playground(object):
    '''
    classdocs
    '''

    def __init__(self, value):
        '''
        Constructor
        '''
        self.vector = [] + value    

    def __setitem__(self, position, item):
        print("setitem", position, item)
        self.vector[position] = item
        
    def __getitem__(self, position):
        print("getitem", position)
        return self.vector[position]
