'''
Created on 10.08.2021

@author: Enzo Brancaccio
'''

import inspect

# equivalent to C++ source_location object
class SourceLocation(object):
    '''
    classdocs
    '''

    # best option seems to be stack from
    # https://docs.python.org/3/library/inspect.html#the-interpreter-stack
    def __init__(self, objToInspect):
        '''
        Constructor
        '''
        self.file_name = inspect.getfile(objToInspect)
        self.function_name
        self.line
        self.column
        