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

from baxter_data_acquisition import settings
from ipl import JointInterpolatedTrajectory


class BangBangInterpolator(JointInterpolatedTrajectory):

    def interpolate(self, q_start, q_end, duration, dq_start, dq_end):
        """ Perform bang bang interpolation. That is, constant acceleration
        from current configuration; motion with constant velocity; constant
        deceleration to end at desired configuration; all optimized to arrive
        there at the desired time.

        :param q_start: Dictionary of joint name keys to start joint angles.
        :param q_end: Dictionary of joint name keys to end joint angles.
        :param duration: Duration for the motion.
        :param dq_start: Dictionary of joint name keys to start joint velocities.
        :param dq_end: Dictionary of joint name keys to end joint velocities.
        :return: (steps, d_steps, err)
        """
        s = None
        ds = None
        err = 0

        for jn in settings.joint_names(self._arm):

            if self._logfile:
                with open(self._logfile, 'a') as fp:
                    fp.write('%s: ' % jn)

            # turn off w2, otherwise duration will grow ridiculously long
            if jn == self._arm+'_w2':
                length = int(np.ceil(duration*settings.interpolator_rate))
                ss, dss, err = np.zeros((length, 1)), np.zeros((length, 1)), 0
                if self._logfile:
                    with open(self._logfile, 'a') as fp:
                        fp.write('\n')
            else:
                ss, dss, err = self._one_dof(T=duration,
                                             q0=q_start[jn], qT=q_end[jn],
                                             dq0=dq_start[jn], dqT=dq_end[jn],
                                             dqm=self._dq_lim[jn],
                                             ddqm=self._ddq_lim[jn])
            if err == -1:
                return (np.zeros((1, len(q_start))),
                        np.zeros((1, len(q_start))), err)

            if s is None:
                s = ss
                ds = dss
            else:
                s = np.concatenate((s, ss), axis=1)
                ds = np.concatenate((ds, dss), axis=1)

        if self._logfile:
            with open(self._logfile, 'a') as fp:
                fp.write('\n')

        return s, ds, err

    def _one_dof(self, T, q0, qT, dq0, dqT, dqm, ddqm):
        if qT < q0:
            dqm = dqm[0]
        else:
            dqm = dqm[1]

        temp = dq0**2 - 2*dq0*dqm + 2*dqm**2 - 2*dqm*dqT + dqT**2
        t2 = -(2*(dq0-dqm)*(q0-qT+dqm*T)) / temp
        t1 = (2*dqT*(q0-qT) + (dq0**2 + dqT**2)*T - 2*dqm*(q0-qT+dq0*T)) / temp
        ddq = temp / (2*(q0 - qT + dqm*T))

        if self._logfile:
            with open(self._logfile, 'a') as fp:
                fp.write('q0=%.4f, qT=%.4f, T=%.1f, dqm=%.2f\n' % (q0, qT, T,
                                                                   dqm))
                fp.write('\tt1=%.2f, t2=%.2f\n' % (t1, t2))
                fp.write('\tddq=%.2f, ddqm=(%.1f, %.1f)\n' % (ddq, ddqm[0],
                                                              ddqm[1]))

        if t1 > t2:
            swap = t1
            t1 = t2
            t2 = swap

        if not 0.0 <= t1 <= t2:
            s = ('something is wrong: ' +
                 '0.00s <= %.2fs <= %.2fs <= %.2fs' % (t1, t2, T))
            self.dbg_msg(s)
            return 0, 0, -1

        if not ddqm[0] <= ddq <= ddqm[1]:
            s = ('required acceleration is out of bounds: ' +
                 '%.2frad/s^2 <= %.2frad/s^2 <= %.2frad/s^2' %
                 (ddqm[0], ddq, ddqm[1]))
            self.dbg_msg(s)
            return 0, 0, -1

        t = np.linspace(0, T, np.ceil(T*settings.interpolator_rate),
                        endpoint=True)
        dq = np.piecewise(t,
                          [t < t1, (t1 <= t) & (t < t2), t >= t2],
                          [lambda tt: dq0 + ddq*tt,
                           lambda tt: dq0 + ddq*t1,
                           lambda tt: dq0 + ddq*(-tt+t1+t2)])
        q = np.piecewise(t,
                         [t < t1, (t1 <= t) & (t < t2), t >= t2],
                         [lambda tt: q0 + dq0*tt + ddq*tt**2/2,
                          lambda tt: q0 + dq0*tt + ddq*t1*tt - ddq*t1**2/2,
                          lambda tt: q0 + dq0*tt - .5*ddq*(tt**2 + t1**2 +
                                                           t2**2 -
                                                           2*(t1+t2)*tt)])

        return (np.asarray(q).reshape((-1, 1)),
                np.asarray(dq).reshape((-1, 1)), 0)
