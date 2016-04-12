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

from hdl import PoseConfigDuration


class DurationHandler(PoseConfigDuration):
    def __init__(self, file_name=None):
        """ A duration handler.
        :param file_name: A file containing a mapping of two configurations to
        the duration required to move between them, where each row and column
        represent a configuration in the list of configurations, and the
        comma-separated entry corresponding to that row and column represents
        the associated duration.
        """
        super(DurationHandler, self).__init__()
        try:
            self._data = self.load_data(file_name=file_name)
        except IOError as e:
            print " => %s" % e
            print "Computing durations mapping ..."
            self._data = self.compute_durations()

    def get_duration(self, closest_idx, des_idx):
        """ Find the duration from closest-to-current to desired configuration
        in the stored duration mapping.
        :param closest_idx: Index of the closest configuration/pose in the
        list of configurations/poses to the current configuration/pose.
        :param des_idx: Index of the desired configuration/pose.
        :return: The stored duration in [s].
        """
        return self._data[closest_idx, des_idx]

    def compute_durations(self):
        """ Compute minimum duration for motions between all combinations of
        configurations in the list of configurations and store them in a file.
        :return: A numpy array containing the mapping.
        """
        import numpy as np

        from baxter_data_acquisition.misc import set_dict
        from baxter_data_acquisition.settings import (
            joint_names,
            dq_scale
        )
        from ipl_bb import BangBangInterpolator

        path = raw_input(" List-of-configurations-file to load: ")
        cfgs = self.load_data(path)

        arm = raw_input(" Compute durations for 'left' or 'right' arm: ")
        if arm not in ['left', 'right']:
            raise ValueError("Must be 'left' or 'right' arm!")

        # Convert list of configs into a list of baxter joint position
        # command dictionaries
        c = [{a: b for a, b in zip(joint_names(arm), cfgs[n, :])}
             for n in range(cfgs.shape[0])]

        bb = BangBangInterpolator(limb=arm, scale_dq=dq_scale, logfile=None,
                                  debug=False)

        # Try, for several durations, to find a solution with the bang-bang
        # interpolator; if successful, store found duration in map.
        durations = np.zeros((cfgs.shape[0], cfgs.shape[0]))
        zeros = set_dict(arm, *(0,)*7)
        times = np.arange(start=0.5, stop=50.0, step=0.5)
        for n in range(cfgs.shape[0]):
            for m in range(n+1, cfgs.shape[0]):
                err = -1
                for t in times:
                    _, _, err = bb.interpolate(q_start=c[n], q_end=c[m],
                                               duration=t,
                                               dq_start=zeros, dq_end=zeros)
                    if err == 0:
                        durations[n, m] = t
                        durations[m, n] = t
                        break
                if err == -1:
                    raise ValueError("No valid trajectory found!")

        path = raw_input(" Where to save the durations mapping: ")
        print "Writing durations into '%s'." % path
        np.savetxt(path, durations, delimiter=',')

        return durations
