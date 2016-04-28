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

from experiments.handshake import Experiment


def main():
    """ (Semi-manual) Data acquisition for handshake-scenario.

    We regard a handshake-scenario between a robot and a human as the simplest
    example of a collaborative task without an object being involved. We wish
    to transform the available (visual) information about the (human) arm into
    a common frame with the (known) robotic arm. That is, learning in this
    case involves (at least) two stages:
    1. Match external and internal information about the controlled (robotic)
       arm.
    2. Match external information about the second ('not controlled' robotic
    or human) arm.

    To this end, we devise the following experiment and record the following
    data. Two hands---either two robotic arms or one robotic and one human
    arm---move repeatedly from the outside toward the inside, 'meeting' at
    some point, indicating (or performing, in case of the second arm being a
    human arm) a handshake. We record three types of data:
    - The internal state of the robotic arm(s).
    - Visual (3d) information from a time-of-flight (TOF) camera mounted on
      the head of the robot, giving a 'egocentric' view of the scene.
    - Visual (RGB and 3d) information from a Kinect V2 sensor standing in
      front of the robot, giving a external view of the scene.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument('-l', '--limb', required=True,
                          choices=['left', 'right'],
                          help='The limb to record data from.')
    required.add_argument('-e', '--experiment', required=True,
                          choices=['r-r', 'r-h'],
                          help='Robot-robot or robot-human handshake.')
    required.add_argument('-m', '--mode', required=True,
                          choices=['normal', 'occluded1', 'occluded2',
                                   'robot', 'human'],
                          help='The set of configurations to select.')
    parser.add_argument('-n', '--number', required=False,
                        type=int, default=1,
                        help='The number of samples to record.')
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
    rospy.init_node('handshake_data', anonymous=True)

    exp = Experiment(limb=args.limb, experiment=args.experiment,
                     number=args.number, threed=args.threed)
    rospy.on_shutdown(exp.clean_shutdown)
    exp.execute(outfile=filename, mode=args.mode)

    print '\nDone.'

if __name__ == '__main__':
    main()
