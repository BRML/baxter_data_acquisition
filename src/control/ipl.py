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

from baxter_data_acquisition.misc import set_dict
from baxter_data_acquisition import settings


class JointInterpolatedTrajectory(object):
    def __init__(self, limb, scale_dq=0.4, logfile=None):
        """ 'Trajectory' refers to a time history of position, velocity, and
        acceleration for each degree of freedom. In joint space, these are
        described as functions of joint angles. Each joint motion can be
        calculated independently from the others, but for proper motion the
        joints should be synchronized, i.e., arrive at their target
        configuration all at the same time. That is, the slowest joint defines
        the velocity of all joints.

        :param limb: Which limb <left, right>.
        :param scale_dq: Scaling factor for joint velocity limits.
        :param logfile: Name of the file to write logfile to, if desired.
        """
        self._arm = limb
        self._logfile = logfile
        self._dq_lim = settings.dq_lim(self._arm, scale=scale_dq)
        self._ddq_lim = settings.ddq_lim(self._arm)

    def interpolate(self, q_start, q_end, duration, dq_start, dq_end):
        """ Implement interpolation scheme here.
        :param q_start: Dictionary of joint name keys to start joint angles.
        :param q_end: Dictionary of joint name keys to end joint angles.
        :param duration: Duration for the motion.
        :param dq_start: Dictionary of joint name keys to start joint velocities.
        :param dq_end: Dictionary of joint name keys to end joint velocities.
        :return: (steps, d_steps, err)
        """
        s = np.zeros((1, 7))
        ds = np.zeros((1, 7))
        err = 0
        return s, ds, err

    def test(self):
        """ Test interpolator by visual inspection. """
        import matplotlib.pyplot as plt
        import pprint
        pp = pprint.PrettyPrinter(indent=2)

        # q_start = set_dict('left', 0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0)
        # q_start = set_dict('left', -1.22, -0.74, -0.11, 1.12, 0.14, 1.2, 3.02)
        # q_start = set_dict('left', -1.23, -0.52, 0.12, 1.25, -0.04, 0.82, 3.02)
        # q_start = set_dict('left', -1.16, -0.31, 0, 1.15, -0.02, 0.66, 3.05)
        # q_start = set_dict('left', -1.23, -0.52, 0.12, 1.25, -0.04, 0.82, 3.02)
        # q_start = set_dict('left', 0.5, -0.7, 0.15, 1.57, -0.28, 0.7, 3.05)
        q_start = set_dict(self._arm, 0.5, -0.35, 0.17, 1.32, -0.33, 0.62, 3.05)
        # q_start = set_dict('left', 0.5, -0.7, 0.15, 1.57, -0.28, 0.7, 3.05)

        q_end = set_dict(self._arm, -0.95, -0.91, -0.14, 1.37, 0.16, 1.12, 3.03)

        dq_start = set_dict(self._arm, *(0.0,)*7)
        dq_end = set_dict(self._arm, *(0.0,)*7)

        print 'q_start:'
        pp.pprint(q_start)
        print 'q_end:'
        pp.pprint(q_end)

        s, ds, err = self.interpolate(q_start=q_start, q_end=q_end,
                                      duration=5.95,
                                      dq_start=dq_start, dq_end=dq_end)

        print 'returned:', err
        print s.shape

        if err == 0:
            q_lim = settings.q_lim(self._arm)
            dq_lim = settings.dq_lim(self._arm, scale=1.0)

            # Visualize trajectory for ocular validation
            fig, axs = plt.subplots(7, 2, figsize=(18, 32), squeeze=False)
            for i, jn in enumerate(settings.joint_names(self._arm)):
                axs[i][0].plot(range(s.shape[0]), s[:, i], 'k-')
                axs[i][0].plot([0, s.shape[0]], [q_start[jn], q_end[jn]], 'ro')
                axs[i][0].axhline(y=q_lim[jn][0], xmin=0, xmax=1,
                                  linewidth=3, color='r', linestyle='dashed',
                                  alpha=.3)
                axs[i][0].axhline(y=q_lim[jn][1], xmin=0, xmax=1,
                                  linewidth=3, color='r', linestyle='dashed',
                                  alpha=.3)
                axs[i][0].set_ylabel('angle [rad]')

                axs[i][1].plot(range(ds.shape[0]), ds[:, i], 'g-')
                axs[i][1].plot([0, ds.shape[0]], [dq_start[jn], dq_end[jn]],
                               'ro')
                axs[i][1].axhline(y=dq_lim[jn][0], xmin=0, xmax=1,
                                  linewidth=3, color='r', linestyle='dashed',
                                  alpha=.3)
                axs[i][1].axhline(y=dq_lim[jn][1], xmin=0, xmax=1,
                                  linewidth=3, color='r', linestyle='dashed',
                                  alpha=.3)
                axs[i][1].set_ylabel('velocity [rad/s]')
            axs[-1][0].set_xlabel('trajectory step')
            axs[-1][1].set_xlabel('trajectory step')
            fig.suptitle('generated trajectory from s0 (top) to w2 (bottom)')
            plt.show()
