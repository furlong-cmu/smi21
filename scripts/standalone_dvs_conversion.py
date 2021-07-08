#!/usr/bin/env python
"""
Program to map camera images to dvs events using v2e
"""

__author__ = 'Michael Furlong'

import numpy as np
import cv2
import rospy
from dvs_msgs.msg import Event, EventArray
from sensor_msgs.msg import Image

from v2ecore.emulator import EventEmulator

import util
import time

class StandaloneDVS:
    
    def __init__(self, module_name=None):
        rospy.init_node(module_name, anonymous=True)
	# Create the event camera emulator.  
	## TODO: Configurate the parameters.
        self.real_generator = True 
        self.emulator = None
        if self.real_generator:
            rospy.loginfo('Creating Event Emulator')
            self.emulator = EventEmulator(
                   pos_thres=0.5,
                   neg_thres=0.5,
                   sigma_thres=0.025,
                   cutoff_hz=0,
                   leak_rate_hz=0,
                   shot_noise_rate_hz=0,
                   device="cpu")
            rospy.loginfo('Emulator created')
        ### end if
        self.events = None

        self.sub = rospy.Subscriber('/husky/husky/camera',
                                    Image,
                                    self.camera_callback)
        self.pub = rospy.Publisher('/smi21/event_image', Image,
                                    queue_size=1)
        self.image = None
        self.num_channels = {'rgb8':3}
	
    def camera_callback(self, img_msg):

        #rospy.loginfo('Image message received')
        colour_img = np.frombuffer(img_msg.data, np.uint8).reshape((img_msg.height, img_msg.width, self.num_channels[img_msg.encoding]))

        num_downsamples = 1 # time between 1 and 2 doesn't seem huge?
        self.image = cv2.cvtColor(colour_img, cv2.COLOR_RGB2GRAY)
        for _ in range(num_downsamples): 
            self.image = cv2.pyrDown(self.image)
        self.time = img_msg.header.stamp.to_sec()

        if self.real_generator:
            rospy.loginfo('Generating events')
            start = time.time()
            self.events = self.emulator.generate_events(self.image, self.time)
            dt = time.time() - start
            rospy.loginfo(f'Events generated in {dt} seconds')
        else:
            self.events = util.random_events(self.image.shape, self.time) 
        ### end if
        event_frame = util.to_event_frame(self.events, self.image.shape)
        zero_channel = np.zeros((event_frame.shape[0], event_frame.shape[1], 1),
                                dtype=np.uint8)

        output = np.concatenate((50 * event_frame, zero_channel), axis=-1).astype(np.uint8)
        msg = Image(height=img_msg.height // 2**num_downsamples,
                    width=img_msg.width // 2**num_downsamples,
                    encoding=img_msg.encoding,
                    is_bigendian=img_msg.is_bigendian,
                    step=img_msg.step//2**num_downsamples,
                    data = output.flatten().tolist())
        msg.header.stamp = rospy.Time.now()
        self.pub.publish(msg)
        #rospy.loginfo('Event Image published')

if __name__ == "__main__":
    m = StandaloneDVS(module_name='dvs_sim')
    rospy.spin()
