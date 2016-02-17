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


""" Data recording settings """

# probability for collision/anomaly [0, 1]
probability = 0.15

# recording rate [Hz]
recording_rate = 500

""" 'Collision' parameters """

# run time per trial [s]
run_time = 30.0


""" 'Anomaly' parameters """


def kpid(limb):
    """
    PID controller parameters for each joint.
    :type limb: string
    :param limb: which limb <left, right>
    :return: Dictionary of joint name keys to pid parameter triples
    """
    if limb not in ['left', 'right']:
        raise ValueError("Only defined for 'left' and 'right' limb!")
    k = dict()
    k[limb + '_s0'] = (19.0, 0.0, 15.0)
    k[limb + '_s1'] = (25.0, 0.0, 16.0)
    k[limb + '_e0'] = (17.0, 0.0, 3.0)
    k[limb + '_e1'] = (7.0, 0.0, 2.0)
    k[limb + '_w0'] = (9.0, 0.2, 2.0)
    k[limb + '_w1'] = (6.0, 0.05, 2.0)
    k[limb + '_w2'] = (3.0, 0.05, 0.8)
    return k

# PID parameters modification limits
pid_mod = {'P': 3.535, 'I': 3.535, 'D': 3.535}


def tau_max(limb):
    """
    Maximum applicable torque per joint [Nm].
    :type limb: string
    :param limb: which limb <left, right>
    :return: Dictionary of joint name keys to torque limit tuples
    """
    if limb not in ['left', 'right']:
        raise ValueError("Only defined for 'left' and 'right' limb!")
    factor = 0.05  # 0.03
    t = dict()
    t[limb + '_s0'] = (-50.0 * factor, 50.0 * factor)
    t[limb + '_s1'] = (-50.0 * factor, 50.0 * factor)
    t[limb + '_e0'] = (-50.0 * factor, 50.0 * factor)
    t[limb + '_e1'] = (-50.0 * factor, 50.0 * factor)
    t[limb + '_w0'] = (-15.0 * factor, 15.0 * factor)
    t[limb + '_w1'] = (-15.0 * factor, 15.0 * factor)
    t[limb + '_w2'] = (-15.0 * factor, 15.0 * factor)
    return t

# interpolator frequency [Hz]
interpolator_rate = 150

# number of anomalous steps, commanded at interpolator_rate
anomal_iters = 75
