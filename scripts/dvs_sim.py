#!/usr/bin/env python
"""
IBA Module example.
This module will run four times during every CLE loop and will output a log message at every iteration.
"""

__author__ = 'Omer Yilmaz'

import rospy


import numpy as np
import cv2
import torch

from dvs_msgs.msg import Event, EventArray
from sensor_msgs.msg import Image

from v2ecore.emulator import EventEmulator

from external_module_interface.external_module import ExternalModule


class DVSSim(ExternalModule):
    
    def __init__(self, module_name=None, steps=1, module_id=123):
        super(DVSSim, self).__init__(module_name, steps)

        self.module_id = module_id

        self.real_generator = False #rospy.get_param('real_generator')
        self.publishing = True # rospy.get_param('publishing')
        self.emulator = None
        if self.real_generator:
            self.emulator = EventEmulator(
                   pos_thres=0.2, #rospy.get_param('pos_thres'),
                   neg_thres=0.2, #rospy.get_param('neg_thres'),
                   sigma_thres=0.03,#rospy.get_param('sigma_thres'),
                   cutoff_hz=200, #rospy.get_param('cutoff_hz'),
                   leak_rate_hz=1,#rospy.get_param('leak_rate_hz'),
                   shot_noise_rate_hz=10,#rospy.get_param('shot_noise_rate_hz'),
                   device=torch.device('cuda' if torch.cuda.is_available() else 'cpu') 
        ### end if self.events = None 

        self.sub = rospy.Subscriber('/husky/husky/camera',
                                    Image,
                                    self.camera_callback)
        if self.publishing:
            self.pub = rospy.Publisher('/smi21/dvs_event_array', EventArray, queue_size=1)
        self.image = None
        self.num_channels = {'rgb8':3}


    
    # def initialize(self):
    #     pass
    
    def run_step(self): 
        rospy.logwarn("DVS Sim called")

        if self.real_generator:
            self.events = self.emulator.generate_events(self.image, self.time)
        else:
            num_events = np.random.randint(10)
            ts = np.ones((num_events, 1)) * self.time
            xs = np.random.randint(low=0,high=self.image.shape[1], size=(num_events,1)).astype('uint16')
            ys = np.random.randint(low=0,high=self.image.shape[0], size=(num_events,1)).astype('uint16')
            ps = (np.random.random((num_events,1)) > 0.5).astype('bool')
            self.events = np.hstack((ts,xs,ys,ps))

    # def shutdown(self):
    #     pass

    def share_module_data(self):
        '''
        note the length of the events should 
        '''
        if not self.events is None:
            self.module_data = [self.module_id]
            self.module_data.extend(self.to_event_frame(self.events).flatten())
            if self.publishing:
                msg = self.to_event_array(self.events)
                self.pub.publish(msg)
            ### end if
            self.events = None
        ### end if
    ### end share_module_data
    
    # Callback functions
    def camera_callback(self, img_msg):

        # TODO:Make sure colour conversion and reshaping works properly
        colour_img = np.frombuffer(img_msg.data, np.uint8).reshape((img_msg.height, img_msg.width, self.num_channels[img_msg.encoding]))

        self.image = cv2.cvtColor(colour_img, cv2.COLOR_RGB2GRAY)
        self.time = img_msg.header.stamp.to_sec()

    # Helper functions
    def to_event_frame(self, events):
        '''
        @param events an N x 4 numpy array with each row being 
            (t, x, y, polarity)
        '''
        retval = np.zeros((self.image.shape[0], self.image.shape[1], 2))
        for e in events:
            # TODO: Confirm that x -> width and y-> height
            retval[int(e[2]),int(e[1]),int(s[3])] += 1
        return retval

    def to_event_array(self, events):

        msg = EventArray()
        msg.header.stamp = timer_event.current_real
        msg.height = self.image.shape[0]
        msg.width = self.image.shape[1]
        msg.events = [Event(ts=rospy.Time.from_sec(x[0]),
                            x=x[1].astype('uint16'),
                            y=x[2].astype('uint16'),
                            polarity=x[3].astype('bool')) for x in self.events]
        return msg


if __name__ == "__main__":
    m = DVSSim(module_name='dvs_sim', steps=1)
    rospy.spin()
