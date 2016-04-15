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
import os
import rospkg
import rospy

from baxter_data_acquisition.jp_anomaly import JointPosition
from baxter_data_acquisition.simulation import sim_or_real


def main():
    """ (Automated) anomaly data acquisition with the baxter research robot.

    A robotic arm is moved between 10 (pseudo-)random poses in Cartesian space in
    a position-controlled manner. To this end, a trajectory between the current
    configuration of the arm and the next desired configuration (corresponding to
    the next desired pose) is computed in a bang-bang-approach (That is, constant
    acceleration from current configuration; motion with constant velocity;
    constant deceleration to end at desired configuration), where acceleration,
    deceleration and constant velocity are chosen such that the desired
    configuration is met in a given desired duration (in seconds), yielding S-
    shaped trajectory segments in joint space. From the seven joints of the
    robotic arm the second wrist joint (w2) is excluded from the PID controller.

    For each of the ten segments of motion (between the 10 poses) there is a chance
    of 15% that an anomaly occurs. Such an anomaly is induced by multiplying the P,
    I and D parameters of the PID controller of one of the joints (sampled at
    random) with independently sampled values from the interval [0, 3.535] for a
    period of 0.5 seconds, returning to the normal P, I and D values afterwards.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument('-l', '--limb', required=True,
                          choices=['left', 'right'],
                          help='The limb to record data from.')
    required.add_argument('-e', '--experiment', required=True,
                          choices=['randomized', 'fixed'],
                          help='The order of positions in the experiment.')
    parser.add_argument('-n', '--number', required=False,
                        type=int, default=1,
                        help='The number of samples to record.')
    parser.add_argument('-a', '--anomalies', required=False,
                        type=bool, default=False,
                        help='Whether there are anomalies in the data.')
    parser.add_argument('-i', '--images', required=False,
                        type=bool, default=False,
                        help='Whether images are to be recorded.')
    parser.add_argument('-t', '--threed', required=False,
                        type=bool, default=False,
                        help='Whether 3d point clouds are to be recorded.')
    parser.add_argument('-o', '--outfile', required=False,
                        type=str, default='',
                        help='Recorded data filename (without extension).')
    args = parser.parse_args(rospy.myargv()[1:])

    ns = rospkg.RosPack().get_path('baxter_data_acquisition')
    datapath = os.path.join(ns, 'data')
    if not os.path.exists(datapath):
        os.makedirs(datapath)
    if len(args.outfile) == 0:
        now = datetime.datetime.now()
        outfile = now.strftime("%Y%m%d%H%M")
    else:
        outfile = args.outfile
    # move all recorded data into a corresponding sub-folder
    filepath = os.path.join(datapath, outfile)
    if not os.path.exists(filepath):
        os.makedirs(filepath)
    filename = os.path.join(filepath, outfile)

    print 'Initializing node ...'
    rospy.init_node('anomaly_data', anonymous=True)

    sim = sim_or_real()
    jp = JointPosition(limb=args.limb, experiment=args.experiment,
                       number=args.number, anomalies=args.anomalies,
                       images=args.images, threed=args.threed, sim=sim)
    rospy.on_shutdown(jp.clean_shutdown)
    jp.execute(filename)

    print '\nDone.'

if __name__ == '__main__':
    main()
