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

import numpy as np
import snappy
import struct


class DepthRecorder(object):
    def __init__(self):
        pass

    def start(self, outname):
        pass

    def stop(self):
        pass


def depth_from_binary(binary_name):
    """ Decode binary file containing depth images and return the depth
    images as a numpy ndarray.
    :param binary_name: The file name of the binary file to read.
    :return: numpy array containing 'l' images of size 240x320.
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
    return images.reshape((l, 240, 320))


if __name__ == '__main__':
    import cv2

    fn = '/home/baxter/Downloads/DepthSenseDepthLog2015-12-17 13.13.19.647.bin'

    images = depth_from_binary(fn)

    for i in range(images.shape[0]):
        cv2.imshow('depthimage', images[i, :, :])
        cv2.waitKey(0)
    cv2.destroyAllWindows()
