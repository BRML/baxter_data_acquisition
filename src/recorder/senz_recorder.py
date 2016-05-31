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


class SenzRecorder(object):
    def __init__(self):
        """ Senz3d sensor recorder class writing RGB images into a .avi
        video file and corresponding time stamps into an accompanying .txt
        file, as well as depth images into a .bin binary file and
        corresponding time stamps into an accompanying .txt file.
        :return:
        """
        self._rec_rgb = CameraRecorder()
        self._rec_rgb.topic = '/senz3d/rgb/image'

        self._rec_depth = DepthRecorder()
        self._rec_depth.topic = '/senz3d/depth/image'

    def start(self, outname):
        """ Set up the Senz3d recorder and record both RGB- and depth data.
        :param outname: Filename to write the RGB- and depth data to, without
        the extension.
        """
        # record rgb image + time stamps
        self._rec_rgb.start(outname=outname + '_rgb',
                            fps=30.0, imgsize=(1280, 720))
        # record depth image + time stamps
        self._rec_depth.start(outname=outname + '_depth')

    def stop(self):
        """ Stop the Senz3d recorder """
        self._rec_depth.stop()
        self._rec_rgb.stop()
