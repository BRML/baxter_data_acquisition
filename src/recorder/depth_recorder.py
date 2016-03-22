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


class DepthRecorder(object):
    def __init__(self):
        """ Depth recorder class writing depth ROS image messages (single
        channel Float32 image) recorded with, e.g., the Kinect V2 sensor,
        compressed into a .bin binary file and timestamps for each image
        frame into an accompanying .txt file.
        """
        self._fp_d = None
        self._fp_ts = None
        self._sub = None
        self._camera = ""
        self.camera = '/cameras/kinect/depth/image_raw'

    def start(self, outname):
        """ Set up the depth recorder with the parameters for the recording
        and subscribe to the callback function of the depth sensor.
        :param outname: Filename to write the binary and text files to,
        without the extension.
        :return: Whether the binary- and text file were opened successfully.
        """
        try:
            self._fp_ts = open(outname + '_depth.txt', 'w')
        except IOError:
            print "ERROR-start-Problem with opening text file."
            raise
        self._fp_ts.write('# timestamps [s]\n')

        try:
            self._fp_d = open(outname + '_depth.bin', 'wb')
        except IOError:
            print "ERROR-start-Problem with opening binary file."
            raise
        self._sub = rospy.Subscriber(self.camera,
                                     Image, callback=self._add_image)
        return not (self._fp_d.closed and self._fp_ts.closed)

    def _add_image(self, imgmsg):
        """ Camera subscriber callback function """
        ts = rospy.get_time()
        self._fp_ts.write('%f\n' % ts)

        try:
            img_float32 = cv_bridge.CvBridge().imgmsg_to_cv2(imgmsg)
        except cv_bridge.CvBridgeError:
            print 'ERROR-add_image-Problem with ROS image message conversion.'
            raise

        # Scale float32 image to uint16 image.
        # '{min, max}_cutoff are the minimum and maximum range of the depth
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

    def stop(self):
        """ Stop recording data from the depth sensor.
        :return: Whether the binary- and text file are open.
        """
        self._sub.unregister()
        self._fp_d.close()
        self._fp_ts.close()
        return self._fp_d.closed or self._fp_ts.closed

    @property
    def camera(self):
        """ String identifying the camera to record images from.
        :return: Camera name.
        """
        return self._camera

    @camera.setter
    def camera(self, camera):
        self._camera = camera


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