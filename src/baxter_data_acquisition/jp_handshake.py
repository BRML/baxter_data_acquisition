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

import math
import numpy.random as rnd
import rospy

from baxter_core_msgs.msg import JointCommand
from std_msgs.msg import (
    Bool,
    Float64MultiArray,
    UInt16
)

import baxter_interface
from baxter_interface import CHECK_VERSION

from baxter_data_acquisition.misc import set_dict
import baxter_data_acquisition.settings as settings

from recorder.joint_recorder import JointRecorder


class JointPosition(object):
    def __init__(self, limb, experiment, number, threed):
        """ Joint position data acquisition for handshake scenario.
        :param limb: The 'robotic' limb to record data from. The other limb
        will be the 'human' limb.
        :param experiment: Two robotic arms (r-r) or one robotic and one
        human arm (r-h) in the experiment.
        :param number: The number of samples to record.
        :param threed: Whether 3d point clouds are to be recorded.
        :return: A baxter robot instance.
        """
        self._arm_robot = limb
        if self._arm_robot == 'left':
            self._arm_human = 'right'
        else:
            self._arm_human = 'left'
        self._experiment = experiment
        self._number = number
        self._threed = threed

        self._limb_robot = baxter_interface.Limb(self._arm_robot)
        self._rec_joint_robot = JointRecorder(limb=self._arm_robot,
                                              rate=settings.recording_rate)
        self._limb_human = baxter_interface.Limb(self._arm_human)
        self._rec_joint_human = JointRecorder(limb=self._arm_human,
                                              rate=settings.recording_rate)
        if self._threed:
            # TODO: set up Kinect recorder instance here
            # TODO: set up RealSense recorder instance here
            pass

        self._pub_rate = rospy.Publisher('robot/joint_state_publish_rate',
                                         UInt16, queue_size=10)
        # TODO: make distinction for 'left' and 'right' arm
        self._pub_cfg_des = rospy.Publisher('data/cfg/des', JointCommand,
                                            queue_size=10)

        print "\nGetting robot state ... "
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print "Enabling robot... "
        self._rs.enable()

        self._limb_robot.set_joint_position_speed(0.3)
        self._pub_rate.publish(settings.recording_rate)

    def clean_shutdown(self):
        """ Clean shutdown of the robot.
        :return: True on completion
        """
        print "\nExiting joint position handshake daq ..."
        self._limb_robot.set_joint_position_speed(0.3)
        self._pub_rate.publish(100)
        self._limb_robot.move_to_neutral()
        if not self._init_state:
            print "Disabling robot..."
            self._rs.disable()
        return True

    def execute(self, outfile, mode):
        """ Recording of handshake data with the baxter research robot.
        :param outfile: path and filename of the file(s) to write the data to,
        without the extension(s).
        """
        print '\nRecord handshake data into %s.' % outfile
        self._limb_robot.move_to_neutral()
        try:
            for nr in range(self._number):
                if rospy.is_shutdown():
                    break
                print 'Recording sample %i of %d.' % (nr + 1, self._number)

                self._rec_joint_robot.start(outfile + '_robot')
                if (self._experiment == 'r-r' or
                        (self._experiment == 'r-h' and mode == 'robot')):
                    self._rec_joint_human.start(outfile + '_human')
                if self._threed:
                    # TODO: start Kinect recorder
                    # TODO: start RealSense recorder
                    pass
                self._one_sample(mode=mode)
                if self._threed:
                    # TODO: stop Kinect recorder
                    # TODO: stop RealSense recorder
                    pass
                self._rec_joint_robot.stop()
                self._rec_joint_robot.write_sample()
                if (self._experiment == 'r-r' or
                        (self._experiment == 'r-h' and mode == 'robot')):
                    self._rec_joint_human.stop()
                    self._rec_joint_human.write_sample()
        except rospy.ROSInterruptException:
            pass
        rospy.signal_shutdown('Done with experiment.')

    def _one_sample(self, mode='normal'):
        """ One handshake sample.
        Baxter moves two limbs in a velocity-controlled sine-wave or one limb
        from an outer- toward an inner configuration and vice versa,
        depending on self._experiment.
        :return: True on completion.
        """
        if self._experiment == 'r-r':
            cmd_robot, cmd_human = self._get_r_r_configurations(mode)

            command = [cmd_robot[jn]
                       for jn in self._rec_joint_robot.get_header_cfg()[1:]]
            # TODO: use robot publisher
            self._pub_cfg_des.publish(command=command)
            self._limb_robot.move_to_joint_positions(cmd_robot)

            command = [cmd_human[jn]
                       for jn in self._rec_joint_human.get_header_cfg()[1:]]
            # TODO: use human publisher
            self._pub_cfg_des.publish(command=command)
            self._limb_human.move_to_joint_positions(cmd_human)

            self._wobble()
        else:  # self._experiment == 'r-h'
            cmd_in, cmd_out = self._get_r_h_configurations()
            if mode == 'robot':
                cmd_human = set_dict(self._limb_human,
                                     -0.11, 0.40, 1.29, 1.62,
                                     -0.19, 0.82, 1.28)
                command = [cmd_human[jn]
                           for jn in self._rec_joint_human.get_header_cfg()[1:]]
                # TODO: use human publisher
                self._pub_cfg_des.publish(command=command)
                self._limb_human.move_to_joint_positions(cmd_human)

            flag_out = False
            elapsed = 0.0
            start = rospy.get_time()
            while not rospy.is_shutdown() and elapsed < settings.run_time:
                if flag_out:
                    cmd = cmd_in
                else:
                    cmd = cmd_out
                command = [cmd[jn]
                           for jn in self._rec_joint_robot.get_header_cfg()[1:]]
                # TODO: use robot publisher
                self._pub_cfg_des.publish(command=command)
                self._limb_robot.move_to_joint_positions(cmd)
                flag_out = not flag_out
                elapsed = rospy.get_time() - start

        self._limb_robot.move_to_neutral()
        return True

    def _wobble(self):
        """ Velocity controlled sine wave motion of both arms around their
        current configuration. The sine wave is randomly parametrized.
        Adapted from
        https://github.com/RethinkRobotics/
          baxter_examples/blob/master/scripts/joint_velocity_wobbler.py
        :return: True on completion
        """
        rate = rospy.Rate(settings.recording_rate)
        jns_robot = settings.joint_names(self._arm_robot)
        jns_human = settings.joint_names(self._arm_human)

        def make_v_func():
            """
            returns a randomly parametrized cosine function to control a
            specific joint.
            """
            period_factor = rnd.uniform(0.3, 0.5)
            amplitude_factor = rnd.uniform(0.1, 0.3)

            def v_func(elapsed):
                w = period_factor*elapsed
                return amplitude_factor*math.cos(w*2*math.pi)
            return v_func

        v_funcs = [make_v_func() for _ in jns_robot]

        def make_cmd(joint_names, elapsed):
            return {jn: v_funcs[i](elapsed)
                    for i, jn in enumerate(joint_names)}

        elapsed = 0.0
        start = rospy.get_time()
        while not rospy.is_shutdown() and elapsed < settings.run_time:
            self._pub_rate.publish(settings.recording_rate)
            elapsed = rospy.get_time() - start
            cmd = make_cmd(jns_robot, elapsed)
            self._limb_robot.set_joint_velocities(cmd)
            cmd = make_cmd(jns_human, elapsed)
            self._limb_human.set_joint_velocities(cmd)
            rate.sleep()
        return True

    def _get_r_r_configurations(self, mode):
        """ Define the two configuration dictionaries for the robotic arm and
        the 'human' arm---which in fact is another robotic arm---in the
        robot-robot-handshake scenario.
        :param mode: The set of configurations to return, one of
        <'normal', 'occluded1', 'occluded2'>.
        :return: Two dictionaries of joint name keys to joint angle values
        defining the configuration for the robotic and the human arm,
        respectively.
        """
        if mode == 'normal':
            return (set_dict(self._arm_robot,
                             0.02, 0.05, -1.16, 1.82, -0.09, 0.41, -0.69),
                    set_dict(self._arm_human,
                             0.29, 0.54, 0.88, 1.22, 0.45, 0.94, 0.50))
        elif mode == 'occluded1':
            return (set_dict(self._arm_robot,
                             0.08, 0.25, -1.24, 2.10, -0.39, 0.48, 1.07),
                    set_dict(self._arm_human,
                             0.29, 0.54, 0.88, 1.22, 0.45, 0.94, 0.50))
        elif mode == 'occluded2':
            return (set_dict(self._arm_robot,
                             -0.12, 0.22, -0.97, 1.85, -0.79, 0.48, 0.23),
                    set_dict(self._arm_human,
                             0.40, 0.37, 0.78, 1.13, 0.69, 1.05, -0.25))
        else:
            raise ValueError('No such set of configurations!')

    def _get_r_h_configurations(self):
        """ Define the inner and outer configuration dictionaries for the
        robotic arm in the robot-human-handshake scenario.
        :return: Two dictionaries of joint name keys to joint angle values
        defining the inner and outer configurations for the robotic arm.
        """
        cmd_in = set_dict(self._arm_robot,
                          0.12, 0.24, -1.23, 1.76, 0.09, 0.61, 1.97)
        cmd_out = set_dict(self._arm_robot,
                           0.57, 0.24, -1.29, 1.53, 0.20, 1.32, 1.97)
        return cmd_in, cmd_out
