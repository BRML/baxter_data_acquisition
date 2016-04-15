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
from tf import transformations

from baxter_core_msgs.msg import JointCommand
from std_msgs.msg import (
    Bool,
    Float64MultiArray,
    UInt16
)

import baxter_interface
from baxter_interface import CHECK_VERSION

from baxter_data_acquisition.face import flash_screen
from baxter_data_acquisition.misc import set_dict
from baxter_data_acquisition.sampler import AnomalySampler
import baxter_data_acquisition.settings as settings
from baxter_data_acquisition.suppression import (
    AvoidanceSuppressor,
    DetectionSuppressor
)

from control import (
    PidController,
    PID_DIRECT,
    BangBangInterpolator,
    ConfigurationHandler,
    DurationHandler
)
from recorder import (
    CameraClient,
    FlashClient,
    JointClient,
    KinectClient
)


class JointPosition(object):
    def __init__(self, limb, experiment, number, anomalies, images, threed,
                 sim):
        """ Joint position data acquisition with automatically induced
        anomalies.
        :param limb: The limb to record data from.
        :param experiment: The order of positions in the experiment.
        :param number: The number of samples to record.
        :param anomalies: Whether there are anomalies in the data.
        :param images: Whether images are to be recorded.
        :param threed: Whether 3d point clouds are to be recorded.
        :param sim: Whether in simulation or reality.
        :return: A baxter robot instance.
        """
        self._arm = limb
        self._experiment = experiment
        self._number = number
        self._anomalies = anomalies
        self._images = images
        self._threed = threed
        self._sim = sim

        ns = rospkg.RosPack().get_path('baxter_data_acquisition')
        path = os.path.join(ns, 'data', 'setup', 'new')
        if not os.path.exists(path):
            os.makedirs(path)
        config_file = os.path.join(path, 'configurations2.txt')
        self._configs = ConfigurationHandler(file_name=config_file)
        duration_file = os.path.join(path, 'durations2.txt')
        self._durations = DurationHandler(file_name=duration_file)
        print np.max(np.max(self._durations))

        path = os.path.join(ns, 'data', 'log')
        if not os.path.exists(path):
            os.makedirs(path)
        log_file = os.path.join(path, 'bang_bang_log.txt')
        rospy.logwarn("Did you modify 'settings.scale_dq'? " +
                      "If yes, re-run 'DurationHandler.compute_durations()'!")
        self._ipl = BangBangInterpolator(limb=self._arm,
                                         scale_dq=settings.dq_scale,
                                         logfile=log_file,
                                         debug=True)

        self._limb = baxter_interface.Limb(self._arm)
        self._rec_joint = JointClient(limb=self._arm,
                                      rate=settings.recording_rate,
                                      anomaly_mode='automatic')
        self._head = baxter_interface.Head()

        if self._images:
            cam = 'head_camera'
            self._camera = baxter_interface.CameraController(cam, self._sim)
            self._rec_cam = CameraClient()
        if self._threed:
            self._rec_kinect = KinectClient()
            self._rec_flash = FlashClient()

        self._pub_rate = rospy.Publisher('robot/joint_state_publish_rate',
                                         UInt16, queue_size=10)
        ns = 'data/limb/' + self._arm
        self._pub_cfg_des = rospy.Publisher(ns + '/cfg/des', JointCommand,
                                            queue_size=10)
        self._pub_cfg_comm = rospy.Publisher(ns + '/cfg/comm', JointCommand,
                                             queue_size=10)
        self._pub_efft_gen = rospy.Publisher(ns + '/efft/gen', JointCommand,
                                             queue_size=10)

        self._previous_config = None
        if self._anomalies:
            self._sampler = AnomalySampler(settings.probability,
                                           **settings.pid_mod)
            self._pub_anom = rospy.Publisher(ns + '/anomaly',
                                             Float64MultiArray, queue_size=10)

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
        threads = [
            AvoidanceSuppressor(self._arm),
            DetectionSuppressor(self._arm)
        ]
        for thread in threads:
            thread.start()

        print '\nRecord data %s anomalies into %s.' % \
              ('with' if self._anomalies else 'without', outfile)
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
                    self._rec_flash.start(outfile + '-%i_flash_white' % nr)
                flash_screen(3, 0.5, 0.5)
                self._one_sample()
                if self._images:
                    self._rec_cam.stop()
                if self._threed:
                    self._rec_kinect.stop()
                    self._rec_flash.stop()
                self._rec_joint.stop()
                self._rec_joint.write_sample()
        except rospy.ROSInterruptException:
            pass
        for thread in threads:
            ret = thread.stop()
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
        idxs = self._select_configurations()
        jns = settings.joint_names(self._arm)
        zeros = set_dict(self._arm, *(0.0,)*7)
        kpid = settings.kpid(self._arm)
        tau_lim = settings.tau_lim(self._arm, scale=0.2)
        for idx in idxs:
            q_des = {a: b for a, b in zip(jns, self._configs[idx])}
            command = [q_des[jn]
                       for jn in self._rec_joint.get_header_cfg()[1:]]
            self._pub_cfg_des.publish(command=command)
            anomaly_pars = None
            if self._anomalies:
                if self._sampler.shall_anomaly():
                    pm = self._sampler.sample_p_multiplier()
                    im = self._sampler.sample_i_multiplier()
                    dm = self._sampler.sample_d_multiplier()
                    add = 0.0
                    joint_id = np.random.randint(0, 7)
                    a_mode = 1  # 1-'Control', 2-'Feedback'
                    a_type = 0  # 0-'Modified', 1-'Removed'
                    anomaly_pars = [pm, im, dm, add, joint_id, a_mode, a_type]
            self._move_to_joint_positions(des_idx=idx, dq_des=zeros,
                                          kpid=kpid, tau_lim=tau_lim,
                                          anomaly_pars=anomaly_pars,
                                          timeout=15.5)

        self._limb.move_to_neutral()
        return True

    def _move_to_joint_positions(self, des_idx, dq_des, kpid, tau_lim,
                                 anomaly_pars=None, timeout=15.0):
        """ (Blocking) PID control of robot limb with anomalies.
        :param des_idx: Index of desired target configuration in list of
        configurations.
        :param dq_des: Dictionary of joint name keys to desired target
        velocities.
        :param kpid: Dictionary of joint name keys to PID control parameter
        triples.
        :param tau_lim: Dictionary of joint name keys to joint torque limit
        tuples.
        :param anomaly_pars: Anomaly parameters
        [pm, im, dm, add, joint_id, a_mode, a_type] or None. If None, no
        anomaly is generated, otherwise an anomaly is introduced according
        to the passed parameters.
        :param timeout: Timeout for motion in [s].
        :return: Boolean <True, False> on completion.
        """
        q_curr = self._limb.joint_angles()
        dq_curr = set_dict(self._arm, *(0.0,)*7)  # self._limb.joint_velocities()
        count = 0
        rate = rospy.Rate(settings.interpolator_rate)

        """ Trajectory planning """
        jns = settings.joint_names(self._arm)
        q_des = {a: b for a, b in zip(jns, self._configs[des_idx])}
        # using closest configuration for look-up of required duration
        closest_idx = self._configs.get_closest_config([q_curr[jn]
                                                        for jn in jns])
        duration = self._durations.get_duration(closest_idx, des_idx)
        duration += settings.duration_offset
        steps, d_steps, err = self._ipl.interpolate(q_start=q_curr,
                                                    q_end=q_des,
                                                    duration=duration,
                                                    dq_start=dq_curr,
                                                    dq_end=dq_des)

        """ Trajectory execution """
        if err == 0:
            print ("Planned trajectory is %i-dimensional and %i steps long." %
                   (steps.shape[1], steps.shape[0]))

            if anomaly_pars is not None:
                """ Set up anomalies """
                pm, im, dm, _, joint_id, _, _ = anomaly_pars
                joint = jns[joint_id]
                margin = 10
                if settings.anomal_iters + 2*margin > steps.shape[0]:
                    anomaly_iters = steps.shape[0] - 2*margin
                else:
                    anomaly_iters = settings.anomal_iters
                anomaly_start = np.random.randint(low=margin,
                                                  high=steps.shape[0]-anomaly_iters)

            """ Set up PID controllers """
            ctrl = dict()
            for jn in q_curr.keys():
                ctrl[jn] = PidController(kpid=kpid[jn], direction=PID_DIRECT)
                ctrl[jn].set_output_limits(minmax=tau_lim[jn])

            """ Do PID control """
            t_elapsed = 0.0
            t_start = rospy.get_time()
            cmd = dict()
            while (not rospy.is_shutdown() and
                   count < steps.shape[0] and
                   t_elapsed < timeout):
                q_curr = self._limb.joint_angles()
                dq_curr = self._limb.joint_velocities()

                if anomaly_pars is not None:
                    """ Anomaly execution """
                    if count == anomaly_start:
                        kp_mod = kpid[joint][0]*pm
                        ki_mod = kpid[joint][1]*im
                        kd_mod = kpid[joint][2]*dm
                        ctrl[joint].set_parameters(kp=kp_mod, ki=ki_mod,
                                                   kd=kd_mod)
                    elif anomaly_start < count < anomaly_start+anomaly_iters:
                        self._pub_anom.publish(data=anomaly_pars)
                    elif count == anomaly_start+anomaly_iters:
                        ctrl[joint].set_parameters(kpid=kpid[joint])

                for jn in q_curr.keys():
                    idx = jns.index(jn)
                    cmd[jn] = ctrl[jn].compute(q_curr[jn], steps[count][idx],
                                               dq_curr[jn])

                command = [steps[count][jns.index(jn)]
                           for jn in self._rec_joint.get_header_cfg()[1:]]
                self._pub_cfg_comm.publish(command=command)
                command = [ctrl[jn].generated
                           for jn in self._rec_joint.get_header_efft()[1:]]
                self._pub_efft_gen.publish(command=command)

                self._limb.set_joint_torques(cmd)

                count += 1
                rate.sleep()
                t_elapsed = rospy.get_time()-t_start
            if count >= steps.shape[0]:
                print "Arrived at desired configuration."
            if t_elapsed >= timeout:
                print "Motion timed out."
            return True
        return False

    def _select_configurations(self):
        """ Select 10 configurations from the list of configurations.
        :return: Indices of 10 selected configurations from the list of
        configurations.
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
        return idxs
