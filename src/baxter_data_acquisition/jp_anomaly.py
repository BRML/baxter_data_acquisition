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

import numpy as np
import os

import rospkg
import rospy

from baxter_core_msgs.msg import JointCommand
from std_msgs.msg import (
    Bool,
    Float64MultiArray,
    UInt16
)

import baxter_interface
from baxter_interface import CHECK_VERSION

from baxter_data_acquisition.sampler import AnomalySampler
import baxter_data_acquisition.settings as settings
from control.ipl_bb import BangBangInterpolator
from control.pid import PID, DIRECT

from recorder.camera_recorder import CameraRecorder
from recorder.joint_recorder import JointRecorder


class JointPosition(object):
    def __init__(self, limb, experiment, number, anomalies, images, threed):
        """ Joint position data acquisition with automatically induced
        anomalies.
        :param limb: The limb to record data from.
        :param experiment: The order of positions in the experiment.
        :param number: The number of samples to record.
        :param anomalies: Whether there are anomalies in the data.
        :param images: Whether images are to be recorded.
        :param threed: Whether 3d point clouds are to be recorded.
        :return: A baxter robot instance.
        """
        self._arm = limb
        self._experiment = experiment
        self._number = number
        self._anomalies = anomalies
        self._images = images
        self._threed = threed

        ns = rospkg.RosPack().get_path('baxter_data_acquisition')
        path = os.path.join(ns, 'data', 'setup')
        if not os.path.exists(path):
            os.makedirs(path)
        config_file = os.path.join(path, 'configurations2.txt')
        self._configs = self._load_configurations(config_file)

        path = os.path.join(ns, 'data', 'log')
        if not os.path.exists(path):
            os.makedirs(path)
        log_file = os.path.join(path, 'bang_bang_log.txt')
        self._ipl = BangBangInterpolator(limb=self._arm, scale_dq=0.4,
                                         logfile=log_file)

        self._limb = baxter_interface.Limb(self._arm)
        self._rec_joint = JointRecorder(limb=self._arm,
                                        rate=settings.recording_rate,
                                        anomaly_mode='automatic')
        if self._images:
            self._camera = baxter_interface.CameraController('head_camera')
            self._rec_cam = CameraRecorder()
        if self._threed:
            # TODO: set up Kinect recorder instance here
            pass

        self._pub_rate = rospy.Publisher('robot/joint_state_publish_rate',
                                         UInt16, queue_size=10)
        self._pub_cfg_des = rospy.Publisher('data/cfg/des', JointCommand,
                                            queue_size=10)

        self._previous_config = None
        if self._anomalies:
            self._sampler = AnomalySampler(settings.probability,
                                           **settings.pid_mod)
            self._pub_anom = rospy.Publisher('data/anomaly', Float64MultiArray,
                                             queue_size=10)

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
                baxter_interface.CameraController('right_hand_camera').close()
            except AttributeError:
                pass
            self._camera.resolution = (1280, 800)
            self._camera.fps = 14

    def clean_shutdown(self):
        """ Clean shutdown of the robot.
        :return: True on completion
        """
        print "\nExiting joint position anomaly daq ..."
        self._limb.set_joint_position_speed(0.3)
        self._pub_rate.publish(100)
        self._limb.move_to_neutral()
        if not self._init_state:
            print "Disabling robot..."
            self._rs.disable()
        return True

    def execute(self, outfile):
        """ Recording of anomaly data with the baxter research robot.
        :param outfile: path and filename of the file(s) to write the data to,
        without the extension(s).
        """
        print '\nRecord data %s anomalies into %s.' % \
              ('with' if self._anomalies else 'without', outfile)
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
        rospy.signal_shutdown('Done with experiment.')

    def _one_sample(self):
        """ One sample with automatically induced anomalies.

        Baxter moves one limb (randomly or in a fixed order, depending on
        self._experiment) between 10 pre-defined configurations. For each
        movement it is randomly selected whether an anomaly is due. If yes,
        the P, I and D parameters of a randomly selected joint (excluding the
        second wrist joint w2) are modified for 0.5 seconds and returned to
        their default values afterward.
        :return: True on completion.
        """
        cmds = self._select_configurations()
        for cmd in cmds:
            command = [cmd[j] for j in self._rec_joint.get_header_cfg()[1:]]
            self._pub_cfg_des.publish(command=command)
            if self._anomalies:
                if self._sampler.shall_anomaly():
                    pm = self._sampler.sample_p_multiplier()
                    im = self._sampler.sample_i_multiplier()
                    dm = self._sampler.sample_d_multiplier()
                    add = 0.0
                    joint_id = np.random.randint(0, 7)
                    joint = self._limb.joint_names[joint_id]
                    a_mode = 1  # 1-'Control', 2-'Feedback'
                    a_type = 0  # 0-'Modified', 1-'Removed'
                    self._pub_anom.publish(data=[pm, im, dm, add, joint_id,
                                                 a_mode, a_type])
                    # PID with anomalies
            else:
                self._move_to_joint_positions()

        self._limb.move_to_neutral()
        return True

    def _move_to_joint_positions(self, q_des, dq_des, kpid, tau_lim,
                                 timeout=15.0):
        q_curr = self._limb.joint_angles()
        dq_curr = self._limb.joint_velocities()
        count = 0
        rate = rospy.Rate(settings.interpolator_rate)

        """ Trajectory planning """
        # TODO: compute duration
        duration = 0.0 + settings.duration_offset
        steps, d_steps, err = self._ipl.interpolate(q_start=q_curr,
                                                    q_end=q_des,
                                                    duration=duration,
                                                    dq_start=dq_curr,
                                                    dq_end=dq_des)

        """ Trajectory execution """
        if err == 0:
            print "Planned trajectory is %i-dimensional and %i steps long." % \
                  (steps.shape[1], steps.shape[0])

            """ Set up PID controllers """
            ctrl = dict()
            for jn in q_des.keys():
                ctrl[jn] = PID(kpid=kpid[jn], direction=DIRECT)
                ctrl[jn].set_output_limits(minmax=tau_lim[jn])
                print jn, ctrl[jn]

            """ Do PID control """
            t_elapsed = 0.0
            t_start = rospy.get_time()
            cmd = dict()
            while (not rospy.is_shutdown() and
                   count < steps.shape[0] and
                   t_elapsed < timeout):
                q_curr = self._limb.joint_angles()
                dq_curr = self._limb.joint_velocities()

                # TODO: implement

                count += 1
                rate.sleep()
                t_elapsed = rospy.get_time()-t_start
            if count >= steps.shape[0]:
                print "Arrived at desired configuration."
            if t_elapsed > timeout:
                print "Motion timed out."
            return True
        return False

    def _move_to_joint_positions_anomaly(self):
        pass

    @staticmethod
    def _load_configurations(filename):
        """ Load previously computed list of configurations.
        :param filename: File to load configurations from.
        :return: A list of configurations.
        """
        return np.array([[float(number)
                          for number in line.split(',')]
                         for line in open(filename, 'r')])

    def _select_configurations(self):
        """ Select 10 configurations from the list of configurations.
        :return: 10 selected configurations.
        """
        if self._experiment == 'randomized':
            print "Randomly selecting 10 poses from workspace:",
            idxs = np.arange(len(self._configs))
            np.random.shuffle(idxs)
            idxs = idxs[:10]
        elif self._experiment == 'fixed':
            print "Selecting 10 poses from workspace:",
            idxs = [50, 47, 26, 27, 9, 61, 13, 55, 15, 37]
        else:
            raise ValueError("'%s' experiment is not implemented!" %
                             self._experiment)
        for idx in idxs:
            print idx,
        print ''
        return self._configs[idxs]
