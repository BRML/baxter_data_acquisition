#!/usr/bin/env python

# Copyright (c) 2016, BRML
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import cv_bridge
import numpy as np
import rospy
import snappy
import struct

from sensor_msgs.msg import Image

from queue_subscriber import QueueSubscriber


class DepthRecorder(object):
    def __init__(self):
        """ Depth recorder class writing depth ROS image messages (single
        channel Float32 image) recorded with, e.g., the Kinect V2 sensor,
        compressed into a .bin binary file and time stamps for each image
        frame into an accompanying .txt file.
        """
        self._topic = '/cameras/kinect/depth/image_raw'
        self._msg_type = Image
        self._fp_d = None
        self._fp_ts = None
        self._sub = None
        self._running = False

    def __str__(self):
        return rospy.get_caller_id()

    def clean_shutdown(self):
        """ Clean shutdown of the depth recorder. """
        if self._sub:
            self._sub.clean_shutdown()
        self._close_binary()
        self._close_file()
        self._running = False

    def _close_binary(self):
        """ Close binary file we wrote depth image frames into. """
        if self._fp_d:
            rospy.loginfo("'%s' Closing binary file ..." % self)
            self._fp_d.close()
            self._fp_d = None
            rospy.loginfo("'%s' ... closed binary file." % self)

    def _close_file(self):
        """ Close text file we wrote time stamps into. """
        if self._fp_ts:
            rospy.loginfo("'%s' Closing text file ..." % self)
            self._fp_ts.close()
            self._fp_ts = None
            rospy.loginfo("'%s' ... closed text file." % self)

    def start(self, outname):
        """ Set up the depth recorder with the parameters for the recording
        and subscribe to the callback function of the depth sensor.
        :param outname: Filename to write the binary and text files to,
        without the extension.
        :return: Whether the binary- and text file were opened successfully.
        """
        if not rospy.is_shutdown():
            if not self._running:
                # open text file
                try:
                    self._fp_ts = open(outname + '.txt', 'w')
                except IOError as e:
                    rospy.logfatal("'%s' Failed to open text file!" % self)
                    raise e
                self._fp_ts.write('# timestamps [s]\n')
                # open binary file
                try:
                    self._fp_d = open(outname + '.bin', 'wb')
                except IOError as e:
                    rospy.logfatal("'%s' Failed to open binary file!" % self)
                    raise e
                # start subscriber
                self._sub = QueueSubscriber(topic=self._topic,
                                            msg_type=self._msg_type,
                                            callback=self._add_image)
                self._running = True
            else:
                rospy.logwarn("'%s' Already running. I do nothing." % self)
            return not (self._fp_d.closed and self._fp_ts.closed)
        return False

    def _add_image(self, stamped_msg):
        """ Camera subscriber callback function """
        ts, imgmsg = stamped_msg
        # write time stamp to file
        self._fp_ts.write('%f\n' % ts)
        self._fp_ts.flush()
        # add frame to binary file
        try:
            img_float32 = cv_bridge.CvBridge().imgmsg_to_cv2(imgmsg)
        except cv_bridge.CvBridgeError as e:
            rospy.logfatal("'%s' Failed to convert ROS image message!" % self)
            raise e
        # Scale float32 image to uint16 image.
        # '{min, max}_cutoff' are the minimum and maximum range of the depth
        # sensor of the Kinect V2.
        min_cutoff = 0.5
        max_cutoff = 4.5
        img_uint16 = 65535*(img_float32 - min_cutoff)/(max_cutoff - min_cutoff)
        img_uint16 = img_uint16.astype(np.uint16, copy=False)

        # compress image with snappy
        img_comp = snappy.compress(img_uint16)
        # write number of bytes of compressed image
        nr_bytes = struct.pack('<L', len(img_comp))
        self._fp_d.write(nr_bytes)
        # write compressed image
        self._fp_d.write(img_comp)
        self._fp_d.flush()

    def stop(self):
        """ Stop recording data from the depth sensor.
        :return: Whether the binary- and text file are open.
        """
        if not rospy.is_shutdown():
            if self._running:
                if self._sub:
                    self._sub.stop()
                    self._sub = None
                self._close_binary()
                self._close_file()
                self._running = False
                return False
            else:
                rospy.logwarn("'%s' Not running. I do nothing." % self)
                return not (self._fp_d.closed and self._fp_ts.closed)
        return False

    @property
    def topic(self):
        """ Get the camera topic to record images from. """
        return self._topic

    @topic.setter
    def topic(self, topic_name):
        """ Set the camera topic to record images from. """
        self._topic = topic_name

    @topic.deleter
    def topic(self):
        """ Delete the camera topic to record images from. """
        del self._topic


def depth_from_binary(binary_name, imgsize=(240, 320)):
    """ Decode binary file containing depth images and return the depth
    images as a numpy ndarray.
    :param binary_name: The file name of the binary file to read.
    :param imgsize: The size (height, width) of each uncompressed image.
    :return: numpy array containing 'l' images of size 'imgsize'.
    """
    images = list()
    with open(binary_name, 'rb') as fp:
        b = fp.read(4)
        while b != '':
            k = struct.unpack('<L', b)[0]
            image_bytes = fp.read(k)
            images.append(snappy.uncompress(image_bytes))
            b = fp.read(4)
    l = len(images)
    images = np.array(images)
    images = np.fromstring(images, dtype=np.dtype('>u2'))
    return images.reshape((l,) + imgsize)


if __name__ == '__main__':
    import cv2

    # fn = '/home/baxter/Downloads/DepthSenseDepthLog2015-12-17 13.13.19.647.bin'
    fn = '/home/baxter/ros_ws/src/baxter_data_acquisition/data/201603221452-0_kinect_depth_depth.bin'

    imgs = depth_from_binary(fn, (424, 512))

    for i in range(imgs.shape[0]):
        cv2.imshow('depthimage', imgs[i, :, :])
        cv2.waitKey(0)
    cv2.destroyAllWindows()
