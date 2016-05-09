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

import argparse
import datetime
import numpy as np
import numpy.random as rnd
import os
import rospkg
import rospy

from std_msgs.msg import UInt16

import baxter_interface
from baxter_interface import CHECK_VERSION

import baxter_data_acquisition.settings as settings
from baxter_data_acquisition.simulation import sim_or_real
from recorder import JointClient


class Robot(object):
    def __init__(self, limb):
        self._arm = limb
        self._jns = settings.joint_names(self._arm)
        self._limb = baxter_interface.Limb(self._arm)
        self._rec_joint = JointClient(limb=self._arm,
                                      rate=settings.recording_rate)

        print "\nGetting robot state ... "
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print "Enabling robot... "
        self._rs.enable()

        self._limb.set_joint_position_speed(0.3)
        self._pub_rate = rospy.Publisher('robot/joint_state_publish_rate',
                                         UInt16, queue_size=10)
        self._pub_rate.publish(settings.recording_rate)

    def clean_shutdown(self):
        """ Clean shutdown of the robot.
        :return: True on completion
        """
        print "\nExiting motion daq ..."
        self._limb.set_joint_position_speed(0.3)
        self._pub_rate.publish(100)
        self._limb.move_to_neutral()
        if not self._init_state:
            print "Disabling robot..."
            self._rs.disable()
        return True

    def position(self, outfile):
        print '\nRecord joint position motion data into %s.' % outfile
        self._limb.move_to_neutral()
        for nr in range(10):
            if rospy.is_shutdown():
                break
            print 'Recording sample %i of 10.' % (nr + 1)

            self._rec_joint.start(outfile=outfile)
            for i in range(10):
                self._limb.move_to_joint_positions(self._configurations(i))
            self._rec_joint.stop()
            self._rec_joint.write_sample()
            self._limb.move_to_neutral()

    def _configurations(self, configuration):
        """ Returns the desired configuration from the list of configurations.
        :type configuration: int
        :param configuration: The id of the configuration [0, 9].
        """
        configs = [{self._arm + '_w0': -0.15071361223754884,
                    self._arm + '_w1': 1.1017816996398926,
                    self._arm + '_w2': 0.059058260266113285,
                    self._arm + '_e0': -1.5562235075317383,
                    self._arm + '_e1': 1.948155598388672,
                    self._arm + '_s0': -0.42798063933105474,
                    self._arm + '_s1': -0.07363107773437501},
                   {self._arm + '_w0': -0.2949078061340332,
                    self._arm + '_w1': 0.9633399336914064,
                    self._arm + '_w2': 0.388864129284668,
                    self._arm + '_e0': -3.0349809853637697,
                    self._arm + '_e1': 1.1198059738769532,
                    self._arm + '_s0': 0.7804127249450684,
                    self._arm + '_s1': -0.9127185677490235},
                   {self._arm + '_w0': 2.319762443829346,
                    self._arm + '_w1': -0.13575729957275393,
                    self._arm + '_w2': 3.052621764404297,
                    self._arm + '_e0': -0.1817767231567383,
                    self._arm + '_e1': 0.779645734552002,
                    self._arm + '_s0': -0.3179175179260254,
                    self._arm + '_s1': 0.4613447214294434},
                   {self._arm + '_w0': 0.16030099215087892,
                    self._arm + '_w1': 1.3261263896118165,
                    self._arm + '_w2': 1.0941117957092286,
                    self._arm + '_e0': 0.4970097747070313,
                    self._arm + '_e1': 1.4645681555603027,
                    self._arm + '_s0': 1.234471037640381,
                    self._arm + '_s1': 0.15186409782714844},
                   {self._arm + '_w0': -1.6037769119018557,
                    self._arm + '_w1': -0.2688301327697754,
                    self._arm + '_w2': 2.1303158167419434,
                    self._arm + '_e0': 0.44485442797851565,
                    self._arm + '_e1': 1.6958157590698244,
                    self._arm + '_s0': 0.898529245477295,
                    self._arm + '_s1': -0.5794612419616699},
                   {self._arm + '_w0': 0.22396119477539064,
                    self._arm + '_w1': 0.6879903825805664,
                    self._arm + '_w2': 3.0530052596008304,
                    self._arm + '_e0': -1.1224904402526856,
                    self._arm + '_e1': 0.8713010865234375,
                    self._arm + '_s0': -1.1393642289001467,
                    self._arm + '_s1': -0.3459126672729492},
                   {self._arm + '_w0': -0.04333495720825196,
                    self._arm + '_w1': 0.39193209085693365,
                    self._arm + '_w2': 3.0537722499938966,
                    self._arm + '_e0': -0.42222821138305666,
                    self._arm + '_e1': 0.9763787703735353,
                    self._arm + '_s0': -0.9840486743041993,
                    self._arm + '_s1': 0.4103398602905274},
                   {self._arm + '_w0': -0.45405831269531255,
                    self._arm + '_w1': 1.5600584594970703,
                    self._arm + '_w2': 2.744675121588135,
                    self._arm + '_e0': -2.059752700579834,
                    self._arm + '_e1': 1.7192089660583498,
                    self._arm + '_s0': 0.04640291878051758,
                    self._arm + '_s1': -0.3087136332092285},
                   {self._arm + '_w0': -0.562587453314209,
                    self._arm + '_w1': 2.0942672682678225,
                    self._arm + '_w2': 1.9573594831054688,
                    self._arm + '_e0': -1.8844953957641604,
                    self._arm + '_e1': 1.7353157643127441,
                    self._arm + '_s0': 0.10469418865356446,
                    self._arm + '_s1': 0.11773302533569337},
                   {self._arm + '_w0': 0.2439029449951172,
                    self._arm + '_w1': 1.459582718005371,
                    self._arm + '_w2': 3.0537722499938966,
                    self._arm + '_e0': -0.10277671267089844,
                    self._arm + '_e1': 1.5608254498901368,
                    self._arm + '_s0': 1.5002332088378907,
                    self._arm + '_s1': -0.04256796681518555}]
        return configs[configuration]

    def velocity(self, outfile):
        print '\nRecord joint velocity motion data into %s.' % outfile
        neutral = {a: b
                   for a, b in zip(self._jns,
                                   [0.02, 0.05, -1.16, 1.82, -0.09, 0.41, -0.69])}
        self._limb.move_to_joint_positions(neutral)

        rate = rospy.Rate(settings.recording_rate)

        def make_v_func():
            """ Create a randomly parametrized cosine function.
            :return: A randomly parametrized cosine function to control a
            specific joint.
            """
            period_factor = rnd.uniform(0.3, 0.5)
            amplitude_factor = rnd.uniform(0.1, 0.3)

            def v_func(elapsed):
                w = period_factor * elapsed
                return amplitude_factor * np.cos(w * 2 * np.pi)

            return v_func

        v_funcs = [make_v_func() for _ in self._jns]

        def make_cmd(joint_names, elapsed):
            """ Create joint velocity command.
            :param joint_names: list of joint names.
            :param elapsed: elapsed time in [s].
            :return: Dictionary of joint name keys to joint velocity commands
            """
            return {jn: v_funcs[i](elapsed)
                    for i, jn in enumerate(joint_names)}

        for nr in range(10):
            if rospy.is_shutdown():
                break
            print 'Recording sample %i of 10.' % (nr + 1)

            self._rec_joint.start(outfile=outfile)
            elapsed = 0.0
            start = rospy.get_time()
            while not rospy.is_shutdown() and elapsed < settings.run_time:
                elapsed = rospy.get_time() - start
                cmd = make_cmd(self._jns, elapsed)
                self._limb.set_joint_velocities(cmd)
                rate.sleep()
            self._rec_joint.stop()
            self._rec_joint.write_sample()
            self._limb.move_to_joint_positions(neutral)


def main():
    """ Record limb motion of the Baxter research robot.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument('-l', '--limb', required=True,
                          choices=['left', 'right'],
                          help='The limb to record data from.')
    required.add_argument('-m', '--mode', required=True,
                          choices=['position', 'velocity', 'torque'],
                          help='The control mode to use.')
    args = parser.parse_args(rospy.myargv()[1:])

    ns = rospkg.RosPack().get_path('baxter_data_acquisition')
    datapath = os.path.join(ns, 'data')
    if not os.path.exists(datapath):
        os.makedirs(datapath)
    now = datetime.datetime.now()
    outfile = now.strftime("%Y%m%d%H%M")
    # move all recorded data into a corresponding sub-folder
    filepath = os.path.join(datapath, outfile)
    if not os.path.exists(filepath):
        os.makedirs(filepath)
    filename = os.path.join(filepath, outfile)

    print 'Initializing node ...'
    rospy.init_node('rec_motion', anonymous=True)
    rospy.sleep(2.0)

    sim = sim_or_real()
    rob = Robot(limb=args.limb)
    rospy.on_shutdown(rob.clean_shutdown)
    if args.mode == 'position':
        rob.position(outfile=filename)
    elif args.mode == 'velocity':
        rob.velocity(outfile=filename)
    elif args.mode == 'torque':
        pass
    else:
        raise ValueError("No control mode '%s'!" % args.mode)

    print '\nDone.'


if __name__ == '__main__':
    main()
