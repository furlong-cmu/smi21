'''
Helper functions for converting between v2e output 
'''

import numpy as np


def to_event_frame(events, image_shape):
    '''
    @param events an N x 4 numpy array with each row being 
        (t, x, y, polarity)
    '''
    retval = np.zeros((image_shape[0], image_shape[1], 2), dtype=np.uint8)
    for e in events:
        # TODO: Confirm that x -> width and y-> height
        retval[int(e[2]),int(e[1]),int(e[3])] += 1
    return retval

def random_events(image_shape, time):

    num_events = np.random.randint(10)
    ts = np.ones((num_events, 1)) * time
    xs = np.random.randint(low=0,high=image_shape[1], size=(num_events,1)).astype('uint16')
    ys = np.random.randint(low=0,high=image_shape[0], size=(num_events,1)).astype('uint16')
    ps = (np.random.random((num_events,1)) > 0.5).astype('bool')
    return np.hstack((ts,xs,ys,ps))
