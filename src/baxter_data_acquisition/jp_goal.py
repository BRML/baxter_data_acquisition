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

import numpy.random as rnd
import rospy

from std_msgs.msg import UInt16

from baxter_core_msgs.msg import (
    EndpointState,
    JointCommand
)

import baxter_interface
from baxter_interface import CHECK_VERSION

import baxter_data_acquisition.settings as settings

from recorder import (
    CameraRecorder,
    JointRecorder
)


class JointPosition(object):
    def __init__(self, limb, number, images, threed, sim):
        """ Joint position data acquisition with goal oriented movements.
        :param limb: The limb to record data from.
        :param number: The number of samples to record.
        :param images: Whether images are to be recorded.
        :param threed: Whether 3d point clouds are to be recorded.
        :param sim: Whether in simulation or reality.
        :return: A baxter robot instance.
        """
        self._arm = limb
        self._number = number
        self._images = images
        self._threed = threed
        self._sim = sim

        self._limb = baxter_interface.Limb(self._arm)
        self._rec_joint = JointRecorder(limb=self._arm,
                                        rate=settings.recording_rate)

        if self._images:
            cam = 'head_camera'
            self._camera = baxter_interface.CameraController(cam, self._sim)
            self._rec_cam = CameraRecorder()
        if self._threed:
            pass

        self._pub_rate = rospy.Publisher('robot/joint_state_publish_rate',
                                         UInt16, queue_size=10)
        ns = 'data/limb/' + self._arm + '/'
        self._pub_cfg_des = rospy.Publisher(ns + 'cfg/des', JointCommand,
                                            queue_size=10)
        self._pub_pose_des = rospy.Publisher(ns + 'pose/des', EndpointState,
                                             queue_size=10)

        # torque control parameters
        self._rate = 1000.0  # Hz
        self._missed_cmds = 20.0  # Missed cycles before triggering timeout

        print "\nGetting robot state ... "
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print "Enabling robot... "
        self._rs.enable()

        self._limb.set_joint_position_speed(0.3)
        self._pub_rate.publish(settings.recording_rate)
        if self._images:
            # Camera handling is one fragile thing...
            try:
                baxter_interface.CameraController('right_hand_camera',
                                                  self._sim).close()
            except AttributeError:
                pass
            self._camera.resolution = (1280, 800)
            self._camera.fps = 14

    def clean_shutdown(self):
        """ Clean shutdown of the robot.
        :return: True on completion
        """
        print "\nExiting joint position goal oriented motion daq ..."
        self._limb.set_joint_position_speed(0.3)
        self._pub_rate.publish(100)
        self._limb.move_to_neutral()
        if not self._init_state:
            print "Disabling robot..."
            self._rs.disable()
        return True

    def execute(self, outfile):
        """ Recording of goal oriented motion data with the baxter research
        robot.
        :param outfile: path and filename of the file(s) to write the data to,
        without the extension(s).
        """
        print '\nRecord goal oriented motion data into %s.' % outfile
        self._limb.move_to_neutral()
        try:
            for nr in range(self._number):
                if rospy.is_shutdown():
                    break
                print 'Recording sample %i of %d.' % (nr + 1, self._number)

                self._rec_joint.start(outfile)
                if self._images:
                    self._rec_cam.start(outfile + '-%i' % nr,
                                        self._camera.fps,
                                        self._camera.resolution)
                self._one_sample()
                if self._images:
                    self._rec_cam.stop()
                self._rec_joint.stop()
                self._rec_joint.write_sample()
        except rospy.ROSInterruptException:
            pass
        finally:
            self._limb.move_to_neutral()
        rospy.signal_shutdown('Done with experiment.')

    def _one_sample(self):
        """ One sample of goal oriented movement.

        Baxter moves one limb from the current configuration to a
        configuration corresponding to a (pseudo)randomly sampled pose from
        the reachable workspace.
        :return: True on completion.
        """
        control_rate = rospy.Rate(self._rate)

        # for safety purposes, set the control rate command timeout.
        # if the specified number of command cycles are missed, the robot
        # will timeout and disable
        self._limb.set_command_timeout((1.0/self._rate)*self._missed_cmds)

        tau_cmd = self._sample_torque()
        duration = self._sample_duration()
        # TODO publish desired torque and duration

        elapsed = 0.0
        start = rospy.get_time()
        while not rospy.is_shutdown() and elapsed < duration:
            if not self._rs.state().enabled:
                rospy.logerr("Joint torque control failed to meet specified "
                             "control rate timeout!")
                break
            self._limb.set_joint_torques(tau_cmd)
            elapsed = rospy.get_time() - start
            control_rate.sleep()
        return True

    def _sample_torque(self):
        """ Sample a random torque vector from the scaled range of torque
        limits for each joint. The torque value is sampled uniform from the
        range of torques for each joint.
        :return: A dictionary of joint name keys to joint torque values [Nm].
        """
        tau_lim = settings.tau_lim(limb=self._arm, scale=settings.tau_scale)
        tau = dict()
        for jn in tau_lim:
            a, b = tau_lim[jn]
            tau[jn] = (b - a)*rnd.random_sample() + a
        print tau
        return tau

    @staticmethod
    def _sample_duration():
        """ Sample duration for torque vector application uniform from a
        reasonable range.
        :return: The torque control duration [s].
        """
        tau_time = settings.tau_duration
        return tau_time*rnd.random_sample()
