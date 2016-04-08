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

# run time per trial [s]
run_time = 30.0

""" 'Anomaly' parameters """

# PID parameters modification limits
pid_mod = {'p_mult': 3.535, 'i_mult': 3.535, 'd_mult': 3.535}

# number of anomalous steps, commanded at interpolator_rate
anomal_iters = 75


# interpolator frequency [Hz]
interpolator_rate = 150

# interpolator duration offset [s]
duration_offset = 1.729  # 2.95

# interpolator maximum joint velocity scale
dq_scale = 0.4  # 0.4


def joint_names(limb):
    """ Get a list of joints in a given limb.
    :param limb: Which limb <left, right>
    :return: list of joint names
    """
    if limb not in ['left', 'right']:
        raise ValueError("Only defined for 'left' and 'right' limb!")
    return [limb+'_'+jn for jn in ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]


def q_lim(limb):
    """ Get joint angle limits per joint [rad] for a given limb.
    :param limb: Which limb <left, right>
    :return: Dictionary of joint name keys to angle limit tuples
    """
    values = [(-1.70167993878, 1.70167993878), (-2.147, 1.047),
              (-3.05417993878, 3.05417993878), (-0.05, 2.618),
              (-3.059, 3.059), (-1.57079632679, 2.094), (-3.059, 3.059)]
    return {a: b for a, b in zip(joint_names(limb), values)}


def dq_lim(limb, scale=0.4):
    """ Get joint velocity limits per joint [rad/s] for a given limb.
    :type limb: string
    :param limb: Which limb <left, right>
    :param scale: Percentage of max value to apply [0, 1]
    :return: Dictionary of joint name keys to maximum joint velocity tuples
    """
    if not 0.0 <= scale <= 1.0:
        raise ValueError("Scale must be in [0, 1]!")
    # values = [(-1.5, 1.5), (-1.5, 1.5), (-1.5, 1.5), (-1.5, 1.5),
    #           (-4.0, 4.0), (-4.0, 4.0), (-4.0, 4.0)]
    values = [(-.75, .75), (-.75, .75), (-.75, .75), (-.75, .75),
              (-1.0, 1.0), (-1.0, 1.0), (-1.0, 1.0)]
    values = [tuple([v*scale for v in val]) for val in values]
    return {a: b for a, b in zip(joint_names(limb), values)}


def ddq_lim(limb):
    """ Get joint acceleration limits per joint [rad/s^2] for a given limb.
    :type limb: string
    :param limb: Which limb <left, right>
    :return: Dictionary of joint name keys to maximum joint acceleration
    tuples
    """
    values = [(-2.0, 2.0), (-2.0, 2.0), (-2.0, 2.0), (-2.0, 2.0),
              (-2.0, 2.0), (-2.0, 2.0), (-2.0, 2.0)]
    return {a: b for a, b in zip(joint_names(limb), values)}


""" PID control parameters """


def kpid(limb):
    """ PID controller parameters for each joint.
    :type limb: string
    :param limb: Which limb <left, right>
    :return: Dictionary of joint name keys to pid parameter triples
    """
    values = [(19.0, 0.0, 15.0), (25.0, 0.0, 16.0),
              (17.0, 0.0, 3.0), (7.0, 0.0, 2.0),
              (9.0, 0.2, 2.0), (6.0, 0.05, 2.0), (3.0, 0.05, 0.8)]
    return {a: b for a, b in zip(joint_names(limb), values)}


def tau_lim(limb, scale=0.2):
    """ Maximum applicable torque per joint [Nm].
    :type limb: string
    :param limb: which limb <left, right>
    :param scale: Percentage of max value to apply [0, 1]
    :return: Dictionary of joint name keys to torque limit tuples
    """
    if not 0.0 <= scale <= 1.0:
        raise ValueError("Scale must be in [0, 1]!")
    values = [(-50.0, 50.0), (-50.0, 50.0),
              (-50.0, 50.0), (-50.0, 50.0),
              (-15.0, 15.0), (-15.0, 15.0), (-15.0, 15.0)]
    values = [tuple([v*scale for v in val]) for val in values]
    return {a: b for a, b in zip(joint_names(limb), values)}


""" 'Seemingly goal oriented movement' parameters """

# scaling factor for joint torques
tau_scale = 0.005

# duration to apply torque control [s]
tau_duration_min = 5.0
tau_duration_max = 30.0
