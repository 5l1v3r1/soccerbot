DEFAULT = 0

def set_default_serializer(sel):
    DEFAULT = sel
    
def serialize(data, sel = DEFAULT):
    if sel == 0:
        pass

import pickle

def _pickle(data):
    ''' Pickle any data type '''
    ret = pickle.dumps(data)
    return ret
 
def _unpickle(pickled):
    ''' unpickle the pickled data back to original '''
    return pickle.loads(pickled)

def _byte_utf8(s):
    ''' Serializes string to bytes using UTF-8''' 
    return s.encode()
 
def _unbyte_utf8(s):
    ''' Decodes bytes to string using UTF-8'''
    return s.decode()