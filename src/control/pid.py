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

from baxter_data_acquisition import settings

DIRECT = 0
REVERSE = 1


class PidController(object):
    def __init__(self, kpid=None, kp=None, ki=None, kd=None, direction=DIRECT):
        """
        PID controller instance.
        :type kp: double
        :param kp: proportional control parameter
        :type ki: double
        :param ki: integral control parameter
        :type kd: double
        :param kd: derivative control parameter
        :type kpid: (double, double, double)
        :param kpid: (kp, ki, kd)
        :param direction: controller direction
        """
        self._kp = None
        self._ki = None
        self._kd = None
        self._outmin = None
        self._outmax = None
        self._direction = DIRECT

        self.generated = 0.0

        self._i_term = 0.0
        self.set_output_limits(outmin=0, outmax=255)
        if kpid:
            self.set_parameters(kpid=kpid)
        elif kp is not None and ki is not None and kd is not None:
            self.set_parameters(kp=kp, ki=ki, kd=kd)
        else:
            raise ValueError('Requires three control parameters or a triple!')

        if direction:
            self.set_controller_direction(direction=direction)

    def __str__(self):
        return ('PID controller with ' +
                'kp=%2.3f, ki=%2.3f, kd=%2.3f ' % (self._kp, self._ki,
                                                   self._kd) +
                'and %2.3f<=output<=%2.3f' % (self._outmin, self._outmax))

    def compute(self, inpt, setpoint, d_inpt):
        """
        Performs the PID computation. Should be called repeatedly.
        :type inpt: double
        :param inpt: current value
        :type setpoint: double
        :param setpoint: desired value
        :type d_inpt: double
        :param d_inpt: derivative of current value
        :return: control output
        """
        error = setpoint - inpt
        self._i_term += self._ki * error
        self.generated = self._i_term
        if self._i_term > self._outmax:
            self._i_term = self._outmax
        elif self._i_term < self._outmin:
            self._i_term = self._outmin

        output = self._kp * error + self._i_term - self._kd * d_inpt
        self.generated += self._kp * error - self._kd * d_inpt
        if output > self._outmax:
            output = self._outmax
        elif output < self._outmin:
            output = self._outmin

        # print 'current error = ', error
        return output

    def set_parameters(self, kpid=None, kp=None, ki=None, kd=None):
        """
        Set control parameters.
        :type kp: double
        :param kp: proportional control parameter
        :type ki: double
        :param ki: integral control parameter
        :type kd: double
        :param kd: derivative control parameter
        :type kpid: (double, double, double)
        :param kpid: (kp, ki, kd)
        """
        if kpid:
            if len(kpid) is not 3:
                raise ValueError("Parameter 'kpid' must be a triple!")
            kp, ki, kd = kpid
        elif kp is None or ki is None or kd is None:
            raise ValueError('Requires three control parameters or a triple!')

        if kp < 0 or ki < 0 or kd < 0:
            raise ValueError('PID parameters must be non-negative!')

        self._kp = kp
        self._ki = ki
        self._kd = kd

        if self._direction is REVERSE:
            self._kp *= -1
            self._ki *= -1
            self._kd *= -1

    def set_output_limits(self, minmax=None, outmin=None, outmax=None):
        """
        Limit output values to a given range.
        :type outmin: double
        :param outmin: lower limit
        :type outmax: double
        :param outmax: upper limit
        :type minmax: (double, double)
        :param minmax: (lower limit, upper limit)
        """
        if minmax:
            if len(minmax) is not 2:
                raise ValueError("Parameter 'minmax' must be a tuple!")
            outmin, outmax = minmax
        elif outmin is None or outmax is None:
            raise ValueError('Requires either two arguments or a tuple!')

        if outmin > outmax:
            raise ValueError('Min limit must be lower than max limit!')

        self._outmin = outmin
        self._outmax = outmax

    def set_controller_direction(self, direction):
        """
        Set the controller direction to 'DIRECT' or 'REVERSE'.
        'DIRECT' means the output will increase when the error is positive.
        'REVERSE' means the opposite.
        :param direction: controller direction
        """
        if direction not in [DIRECT, REVERSE]:
            raise ValueError("Direction must be 'DIRECT' or 'REVERSE'!")
        if direction is not self._direction:
            self._kp *= -1
            self._ki *= -1
            self._kd *= -1
        self._direction = direction

    def get_kp(self):
        """
        Get current proportional control parameter.
        :return: proportional control parameter
        """
        return self._kp

    def get_ki(self):
        """
        Get current integral control parameter.
        :return: integral control parameter
        """
        return self._ki

    def get_kd(self):
        """
        Get current derivative control parameter.
        :return: derivative control parameter
        """
        return self._kd

    def get_direction(self):
        """
        Get controller direction.
        :return: controller direction
        """
        return 'DIRECT' if self._direction == DIRECT else 'REVERSE'


if __name__ == '__main__':
    import pprint

    limb = 'left'
    joint = limb + '_s0'

    print 'test PID parameters'
    pp = pprint.PrettyPrinter(indent=2)
    pp.pprint(settings.kpid(limb))

    print '\ntest torque limits'
    tau_lim = settings.tau_lim(limb, scale=0.05)
    pp.pprint(tau_lim)

    print '\nset up PID controller'
    pid = PidController(direction=DIRECT, kpid=settings.kpid(limb)[joint])
    print 'kp: ', pid.get_kp(), ' ki: ', pid.get_ki(), ' kd: ', pid.get_kd()
    print 'direction: ', pid.get_direction()
    pid.set_output_limits(minmax=tau_lim[joint])
    print pid

    print '\nset up second PID controller'
    kp, ki, kd = settings.kpid(limb)[joint]
    pid2 = PidController(direction=REVERSE, kp=kp, ki=ki, kd=kd)
    print 'direction: ', pid2.get_direction()
    outmin, outmax = tau_lim[joint]
    pid2.set_output_limits(outmin=outmin, outmax=outmax)
    print pid2

    print ''
    start = 0.0
    res = start
    for idx in range(0, 15):
        start = res
        cmd = pid.compute(start, 10.0, 0.0)
        res = start + cmd
        print 'iteration %2i start = %5.2f command = %5.2f result = %5.2f' % \
              (idx + 1, start, cmd, res)
