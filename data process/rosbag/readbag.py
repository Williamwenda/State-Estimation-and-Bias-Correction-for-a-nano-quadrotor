#!/usr/bin/env python

## it is incomplete.... can not be used

from __future__ import print_function

import sys

import rospy
import rosbag
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

import os       # for file management make directory
import shutil   # for file management, copy file
import scipy.io as sio
import time
import string

import IPython

def stamp_to_float(stamp):
    ''' Convert a ros timestamp message to a float in seconds. '''
    return rospy.Time(secs=stamp.secs, nsecs=stamp.nsecs).to_time()


def align_msgs(msgs1, msgs2):
    ''' Align messages based on time stamp. Messages must have a header.stamp
        field. msgs2 are aligned with msgs1. '''
    i1 = i2 = 0
    aligned_msgs = []
    for i1 in xrange(len(msgs1)):
        t = stamp_to_float(msgs1[i1].header.stamp)
        while i2 + 1 < len(msgs2) and stamp_to_float(msgs2[i2 + 1].header.stamp) < t:
            i2 += 1
        aligned_msgs.append((msgs1[i1], msgs2[i2]))
    return aligned_msgs


class LogDecoder:
        def __init__(self):
                self.values = []
                self.time = []

        def record(self, msg):
                for log in msg.logdata:

                        self.values.append(list(log.values))
                        self.time.append(log.header.stamp.secs + log.header.stamp.nsecs * 1e-9)

        def saveAll(self, dir, name):
                folder = dir + "/log"
                try:  # else already exists
                        os.makedirs(folder)
                except:
                        pass
                sio.savemat(folder + "/" + name +".mat", {name: self.values, "time":self.time})



def main():
    bag = rosbag.Bag(sys.argv[1])
    
    ## Read message from the bag
    imu_msgs      =  [msg for _, msg, _ in bag.read_messages('/log/IMU_Vx')]
    kalmanS_msgs  =  [msg for _, msg, _ in bag.read_messages('/log/kalman_state')]
    flowdeck_msgs =  [msg for _, msg, _ in bag.read_messages('/log/flowdeck_Vy')]
    ViconZ_msgs   =  [msg for _, msg, _ in bag.read_messages('/log/ViconZ_V')]

    #  Here the flowdeck topic publishes fast, align every topics with respect to flowdeck topic
    imu_msgs_aligned = [msg for _, msg in align_msgs(flowdeck_msgs, imu_msgs)] 
    kalmanS_msgs_aligned = [msg for _, msg in align_msgs(flowdeck_msgs, kalmanS_msgs)] 
    ViconZ_msgs_aligned =  [msg for _, msg in align_msgs(flowdeck_msgs, ViconZ_msgs] 
    
##  sio.savemat(folder + "/" + name +".mat", {name: self.values, "time":self.time})
    N = len(flowdeck_msgs)
    l = 0
    u = N
    fig = plt.figure()
    ax = fig.add_subplot
    ax.plot()


if __name__ == '__main__':
    main()

























