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

from baxter_core_msgs.msg import JointCommand

import baxter_interface
from baxter_interface import CHECK_VERSION

from baxter_data_acquisition.face import flash_screen
import baxter_data_acquisition.settings as settings
from baxter_data_acquisition.workspace import Workspace

from recorder import (
    CameraClient,
    FlashClient,
    JointRecorder,
    KinectClient,
    SenzClient
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
        self._head = baxter_interface.Head()

        if self._images:
            cam = 'head_camera'
            self._camera = baxter_interface.CameraController(cam, self._sim)
            self._rec_cam = CameraClient()
        if self._threed:
            self._rec_senz3d = SenzClient()
            self._rec_kinect = KinectClient()
            self._rec_flash = FlashClient()

        self._pub_rate = rospy.Publisher('robot/joint_state_publish_rate',
                                         UInt16, queue_size=10)
        ns = 'data/limb/' + self._arm + '/'
        self._pub_efft_des = rospy.Publisher(ns + 'efft/des', JointCommand,
                                             queue_size=10)
        self._pub_pose_label = rospy.Publisher(ns + 'pose/label', UInt16,
                                               queue_size=10)

        # torque control parameters
        self._rate = 1000.0  # Hz
        self._missed_cmds = 20.0  # Missed cycles before triggering timeout

        # robot workspace using default parameters
        self._ws = Workspace()

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
        self._head.set_pan(0.0)
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
                if self._threed:
                    self._rec_kinect.start(outfile + '-%i_kinect' % nr)
                    self._rec_senz3d.start(outfile + '-%i_senz3d' % nr)
                    self._rec_flash.start(outfile + '-%i_flash_white' % nr)
                flash_screen(3, 0.5, 0.5)
                self._one_sample()
                if self._images:
                    self._rec_cam.stop()
                if self._threed:
                    self._rec_kinect.stop()
                    self._rec_senz3d.stop()
                    self._rec_flash.stop()
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

        elapsed = 0.0
        start = rospy.get_time()
        while not rospy.is_shutdown() and elapsed < duration:
            if not self._rs.state().enabled:
                rospy.logerr("Joint torque control failed to meet specified "
                             "control rate timeout!")
                break
            self._pub_efft_des.publish(
                command=[tau_cmd[jn]
                         for jn in self._rec_joint.get_header_efft()[1:]]
            )
            self._limb.set_joint_torques(tau_cmd)
            elapsed = rospy.get_time() - start
            control_rate.sleep()
        # final position of the end effector defines label of the trajectory
        pose = self._endpoint_pose()
        label = self._ws.cluster_position(pose[:3])
        self._pub_pose_label.publish(label)
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
        return tau

    @staticmethod
    def _sample_duration():
        """ Sample duration for torque vector application uniform from a
        reasonable range.
        :return: The torque control duration [s].
        """
        duration = (
            (settings.tau_duration_max - settings.tau_duration_min) *
            rnd.random_sample() +
            settings.tau_duration_min
        )
        return duration

    def _endpoint_pose(self):
        """ Current pose of the wrist of one arm of the baxter robot.
        :return: pose [px, py, pz, or, op, oy, ow]
        """
        qp = self._limb.endpoint_pose()
        return [
            qp['position'][0], qp['position'][1], qp['position'][2],
            qp['orientation'][0], qp['orientation'][1],
            qp['orientation'][2], qp['orientation'][3]
        ]
