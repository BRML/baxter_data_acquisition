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

from recorder import CameraRecorder
from recorder.depth_recorder import DepthRecorder


class DepthSensorRecorder(object):
    def __init__(self, rgb_image_topic, depth_image_topic, rgb_size, rgb_fps):
        """ Depth sensor recorder class writing RGB images into an .avi video
        file and corresponding time stamps into an accompanying .txt file, as
        well as depth images into a .bin binary file and corresponding time
        stamps into an accompanying .txt file.
        :param rgb_image_topic: ROS topic to get the depth sensor rgb images
            from.
        :param depth_image_topic: ROS topic to get the depth sensor depth
            images from.
        :param rgb_size: The depth sensor rgb image size (width, height).
        :param rgb_fps: The depth sensor rgb frames per second [Hz].
        """
        self._rec_rgb = CameraRecorder()
        self._rec_rgb.topic = rgb_image_topic
        if isinstance(rgb_size, tuple) and len(rgb_size) == 2:
            self._rgb_size = rgb_size
        else:
            raise ValueError("RGB image size must be a tuple!")
        self._rgb_fps = rgb_fps

        self._rec_depth = DepthRecorder()
        self._rec_depth.topic = depth_image_topic

    def start(self, outname):
        """ Set up the the depth sensor recorder and record both RGB- and
        depth data.
        :param outname: Filename to write the RGB- and depth data to, without
            the extension.
        """
        # record rgb image + time stamps
        self._rec_rgb.start(outname=outname + '_rgb',
                            fps=self._rgb_fps, imgsize=self._rgb_size)
        # record depth image + time stamps
        self._rec_depth.start(outname=outname + '_depth')

    def stop(self):
        """ Stop the depth sensor recorder """
        self._rec_depth.stop()
        self._rec_rgb.stop()
