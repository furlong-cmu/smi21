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

class StandaloneDVS:
    
    def __init__(self, module_name=None):
        rospy.init_node(module_name, anonymous=True)
	# Create the event camera emulator.  
	## TODO: Configurate the parameters.
        self.real_generator = False
        self.emulator = None
        if self.real_generator:
            rospy.loginfo('Creating Event Emulator')
            self.emulator = EventEmulator(
                   pos_thres=0.2,
                   neg_thres=0.2,
                   sigma_thres=0.03,
                   cutoff_hz=200,
                   leak_rate_hz=1,
                   shot_noise_rate_hz=10,
                   device="cpu")
        ### end if
        self.events = None

        self.sub = rospy.Subscriber('/husky/husky/camera',
                                    Image,
                                    self.camera_callback)
#        self.pub = rospy.Publisher('/smi21/dvs_event_array', EventArray, queue_size=1)
        self.pub = rospy.Publisher('/smi21/event_image', Image, queue_size=1)
#        rospy.Timer(rospy.Duration(0.1), self.send_event)
        self.image = None
        self.num_channels = {'rgb8':3}
	
    def camera_callback(self, img_msg):

        # TODO:Make sure colour conversion and reshaping works properly
        #rospy.loginfo('Image message received')
        colour_img = np.frombuffer(img_msg.data, np.uint8).reshape((img_msg.height, img_msg.width, self.num_channels[img_msg.encoding]))

        self.image = cv2.cvtColor(colour_img, cv2.COLOR_RGB2GRAY)
        self.time = img_msg.header.stamp.to_sec()

        if self.real_generator:
            self.events = self.emulator.generate_events(self.image, self.time)
        else:
            num_events = np.random.randint(10)
            ts = np.ones((num_events, 1)) * self.time
            xs = np.random.randint(low=0,high=self.image.shape[1], size=(num_events,1)).astype('uint16')
            ys = np.random.randint(low=0,high=self.image.shape[0], size=(num_events,1)).astype('uint16')
            ps = (np.random.random((num_events,1)) > 0.5).astype('bool')
            self.events = np.hstack((ts,xs,ys,ps))
        ### end if
        event_frame = util.to_event_frame(self.events, self.image.shape)
        zero_channel = np.zeros((event_frame.shape[0], event_frame.shape[1], 1),
                                dtype=np.uint8)

        output = np.concatenate((event_frame, zero_channel), axis=-1).astype(np.uint8)
        msg = Image(height=img_msg.height,
                    width=img_msg.width,
                    encoding=img_msg.encoding,
                    data = output.flatten().tolist())
        msg.header.stamp = rospy.Time.now()
        self.pub.publish(msg)
        #rospy.loginfo('Event Image published')






    def send_event(self, timer_event):

	# Because producing events from the first image takes much
	# longer than subsequent images, we don't publish events
	# until we've done the first image, so people won't be getting
	# empty event messages that don't mean anything.
	# (unless we want to add some random noise or something?)
        if not (self.events is None):
            # Events are an N x 4 matrix of (time, x, y, polarity)

            msg = EventArray()
            msg.header.stamp = timer_event.current_real
            msg.height = self.image.shape[0]
            msg.width = self.image.shape[1]
            msg.events = [Event(ts=rospy.Time.from_sec(x[0]), x=x[1].astype('uint16'), y=x[2].astype('uint16'), polarity=x[3].astype('bool')) for x in self.events]

            self.pub.publish(msg)

            # Remove the just processed events.
            ## HACK: Make sure this isn't stupid (re: concurrency) later.
            self.events = None

if __name__ == "__main__":
    m = StandaloneDVS(module_name='dvs_sim')
    rospy.spin()
