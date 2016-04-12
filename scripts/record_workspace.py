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
import rospy

import baxter_interface
from baxter_interface import CHECK_VERSION

from control import PoseHandler, ConfigurationHandler


class Robot(object):
    def __init__(self):
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        print "\nGetting robot state ... "
        self._init_state = self._rs.state().enabled
        print "Enabling robot... "
        self._rs.enable()

    def disable(self):
        if not self._init_state:
            print "Disabling robot..."
            self._rs.disable()


def record(filename):
    """ Move limb manually to record corners of the real robot's workspace.
    :param filename: A file containing poses to load for testing.
    """
    ph = PoseHandler(filename)
    key = raw_input("Test workspace corners? (y, n): ")
    if key == 'y' or key == 'Y':
        ph.test_poses()


def sample(filename):
    """ Sample configurations from within the real robot's workspace.
    :param filename: A file containing poses defining the workspace corners.
    """
    ph = PoseHandler(filename)
    ph.sample()


def test(filename):
    """ Test configurations by moving through them one after the other.
    :param filename: A file containing configurations, one per row.
    """
    ch = ConfigurationHandler(filename)
    ch.test_configs()


def main():
    """ Record poses defining the corners of the workspace of one limb of the
    robot or sample and store configurations from within the workspace.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument('-m', '--mode', required=True,
                          choices=['record', 'sample', 'test'],
                          help='The mode to execute.')
    parser.add_argument('-f', '--file', required=False,
                        type=str, default="",
                        help='A file containing poses to load and test.')
    args = parser.parse_args(rospy.myargv()[1:])

    print 'Initializing node ...'
    rospy.init_node('workspace_rec_smpl', anonymous=True)

    rob = Robot()
    rospy.on_shutdown(rob.disable)

    if args.mode == 'record':
        record(filename=args.file)
    elif args.mode == 'sample':
        sample(filename=args.file)
    else:
        test(filename=args.file)

    print '\nDone.'


if __name__ == '__main__':
    main()
