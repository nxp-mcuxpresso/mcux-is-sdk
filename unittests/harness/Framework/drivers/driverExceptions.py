'''
Created on Jan 3, 2012

@author: R7AACD
'''

class DriverError(Exception):
    '''
    BaseClass for Driver Error
    '''
    pass

class CommError(DriverError):
    '''
    BaseClass for Driver Error
    '''

    def __init__(self, msg):
        '''
        Constructor
        '''
        self.msg = msg

    def __str__(self):
        return repr(self.msg)