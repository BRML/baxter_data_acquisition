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


class ConfigurationHandler(PoseConfigDuration):
    def __init__(self, file_name=None):
        """ A configuration handler.
        :param file_name: A file containing a list of configurations, where
        each row contains a configuration as comma-separated entries.
        """
        super(ConfigurationHandler, self).__init__()
        try:
            self._data = self.load_data(file_name=file_name)
        except IOError as e:
            print " => %s" % e
            print "Computing configurations ..."
            self._data = self.compute_configs()

    def get_closest_config(self, config):
        """ Find the closest configuration from the list of configurations to
        a given configuration. Use L1 norm (sum of absolute distances) as
        metric.
        :param config: The configuration to find the closest configuration to.
        :return: The index of the closest configuration in the list of
        configurations.
        """
        if not isinstance(config, list) and len(config) != 7:
            raise ValueError("Configuration must be a list with 7 entries!")
        try:
            err = map(lambda x: np.sum(abs(x)), self._data - config)
        except Exception:
            raise
        err = np.asarray(err)
        return np.argmin(err)

    def compute_configs(self):
        """ Compute configurations from a list of poses and store them in a
        file.
        :return: numpy array containing the list of configurations.
        """
        path = raw_input(" List-of-poses-file to load: ")
        poses = self.load_data(path)
        arm = raw_input(" Compute poses for 'left' or 'right' arm: ")
        if arm not in ['left', 'right']:
            raise ValueError("Must be 'left' or 'right' arm!")
        cfgs = np.empty((poses.shape[0], 7))
        for nr in range(poses.shape[0]):
            try:
                cfg = self._inverse_kinematics(pose=poses[nr, :], arm=arm)
            except (ValueError, Exception):
                raise
            cfgs[nr, :] = cfg

        path = raw_input(" Where to save the list of configurations: ")
        print "Writing configurations into '%s'." % path
        np.savetxt(path, cfgs, delimiter=',')

        return cfgs
