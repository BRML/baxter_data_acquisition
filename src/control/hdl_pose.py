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

from hdl import PoseConfigDuration


class PoseHandler(PoseConfigDuration):
    def __init__(self, file_name=None):
        """ A pose handler.
        :param file_name: A file containing a list of poses, where each row
        contains a pose as comma-separated entries.
        """
        super(PoseHandler, self).__init__()
        try:
            self._data = self.load_data(file_name=file_name)
        except IOError as e:
            print " => %s" % e
            print "Recording poses ..."
            self._data = self.record_poses()

    def get_closest_pose(self, pose):
        """ Find the closest pose from the list of poses to a given pose. Use
        Euclidean distance as metric.
        :param pose: The pose to find the closest pose to.
        :return: The index of the closest pose in the list of poses.
        """
        if not isinstance(pose, list) and len(pose) != 6:
            raise ValueError("Pose must be a list with 6 entries!")
        try:
            err = map(lambda x: np.sum(x**2), self._data - pose)
        except Exception:
            raise
        err = np.asarray(err)
        return np.argmin(err)

    def record_poses(self):
        # TODO: implement recording of poses and conversion to config
        pass
