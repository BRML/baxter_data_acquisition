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
from scipy.spatial import Delaunay

from std_msgs.msg import (
    Float64MultiArray,
    UInt16
)

from baxter_core_msgs.msg import JointCommand

import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_pykdl import baxter_kinematics

from baxter_data_acquisition.face import flash_screen
from baxter_data_acquisition.misc import set_dict
from baxter_data_acquisition.sampler import AnomalySampler
import baxter_data_acquisition.settings as settings
from baxter_data_acquisition.workspace import Workspace
from control import (
    BangBangInterpolator,
    PidController,
    PID_DIRECT,
    sample_from_workspace
)
from recorder import (
    CameraClient,
    FlashClient,
    JointClient,
    KinectClient,
    SenzClient
)


class Experiment(object):
    def __init__(self, limb, number, images, threed, sim):
        """ Joint position data acquisition with apparent goal oriented
        movements.
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
        self._rec_joint = JointClient(limb=self._arm,
                                      rate=settings.recording_rate,
                                      anomaly_mode='automatic')
        self._head = baxter_interface.Head()
        self._jns = settings.joint_names(self._arm)
        self._ipl = BangBangInterpolator(limb=self._arm,
                                         scale_dq=settings.dq_scale,
                                         logfile=None,
                                         debug=True)
        ns = rospkg.RosPack().get_path('baxter_data_acquisition')
        path = os.path.join(ns, 'data', 'setup', 'new', 'poses.txt')
        self._hull = Delaunay(np.loadtxt(path, delimiter=',')[:, :3])
        self._kin = baxter_kinematics(self._arm)
        self._lim = [settings.q_lim(self._arm)[jn] for jn in self._jns]

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
        self._pub_cfg_des = rospy.Publisher(ns + 'cfg/des', JointCommand,
                                            queue_size=10)

        self._pub_cfg_comm = rospy.Publisher(ns + 'cfg/comm', JointCommand,
                                             queue_size=10)
        self._pub_efft_gen = rospy.Publisher(ns + 'efft/gen', JointCommand,
                                             queue_size=10)
        self._pub_efft_des = rospy.Publisher(ns + 'efft/des', JointCommand,
                                             queue_size=10)
        self._pub_pose_label = rospy.Publisher(ns + 'pose/label', UInt16,
                                               queue_size=10)
        self._pub_anom = rospy.Publisher(ns + 'anomaly',
                                         Float64MultiArray, queue_size=10)
        self._sampler = AnomalySampler(settings.probability,
                                       **settings.pid_mod)

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
        # Get current configuration q0
        q0 = self._limb.joint_angles()
        # Sample pose qT from workspace. Find corresponding configuration.
        # Compute trajectory from q0 to qT.
        qT = None
        trajectory = None
        failed = True
        while failed:
            qT = self._sample_config()
            try:
                trajectory = self._plan_trajectory(q0=q0, qT=qT)
                failed = False
            except ValueError as e:
                print e
        # Prepare anomaly parameters
        anomaly_pars = None
        if self._sampler.shall_anomaly():
            pm = self._sampler.sample_p_multiplier()
            im = self._sampler.sample_i_multiplier()
            dm = self._sampler.sample_d_multiplier()
            add = 0.0
            a_mode = 1  # 1-'Control', 2-'Feedback'
            a_type = 0  # 0-'Modified', 1-'Removed'
            # random sample joint with motion > 10.0 deg, if none is
            # found, no anomaly will be induced!
            for i in range(len(self._jns)):
                if rospy.is_shutdown():
                    break
                joint_id = np.random.randint(0, len(self._jns))
                jn = self._jns[joint_id]
                if abs(q0[jn] - qT[jn]) > np.deg2rad(10.0):
                    anomaly_pars = [pm, im, dm, add, joint_id,
                                    a_mode, a_type]
                    break
        # Report desired configuration and and execute trajectory
        command = [qT[jn] for jn in self._rec_joint.get_header_cfg()[1:]]
        self._pub_cfg_des.publish(command=command)
        self._run_trajectory(trajectory, anomaly_pars)
        # final position of the end effector defines label of the trajectory
        pose = self._endpoint_pose()
        label = self._ws.cluster_position(pose[:3])
        self._pub_pose_label.publish(label)
        return True

    def _sample_config(self):
        """ Sample a pose from the workspace and return the corresponding
        configuration.
        :return: Dictionary of joint name keys to joint angle values.
        """
        config = None
        while config is None:
            try:
                pose, config = \
                    sample_from_workspace(hull=self._hull, kin=self._kin,
                                          lim=self._lim, arm=self._arm)
            except ValueError as e:
                print e
        return {a: b for a, b in zip(self._jns, config)}

    def _plan_trajectory(self, q0, qT):
        """ Plan a trajectory from configuration q0 to configuration qT
        :param q0: Dictionary of joint name keys to current joint angle values.
        :param qT: Dictionary of joint name keys to desired joint angle values.
        :return: A trajectory (Nx7 numpy array).
        :raise: ValueError if no valid trajectory is found.
        """
        zeros = set_dict(self._arm, *(0.0,)*7)
        for t in np.arange(start=0.5, stop=settings.run_time, step=0.15):
            steps, _, err = self._ipl.interpolate(q_start=q0,
                                                  q_end=qT,
                                                  duration=t,
                                                  dq_start=zeros,
                                                  dq_end=zeros)
            if err == 0:
                return steps
        raise ValueError("No valid trajectory found!")

    def _run_trajectory(self, trajectory, anomaly_pars=None):
        """ Execute trajectory, possibly with anomalies.
        :param trajectory: A trajectory (Nx7 numpy array).
        :param anomaly_pars: Anomaly parameters
        [pm, im, dm, add, joint_id, a_mode, a_type] or None. If None, no
        anomaly is generated, otherwise an anomaly is introduced according
        to the passed parameters.
        """
        kpid = settings.kpid(self._arm)
        tau_lim = settings.tau_lim(self._arm, scale=0.2)
        count = 0
        rate = rospy.Rate(settings.interpolator_rate)
        print ("Planned trajectory is %i-dimensional and %i steps long." %
               (trajectory.shape[1], trajectory.shape[0]))

        if anomaly_pars is not None:
            """ Set up anomalies """
            pm, im, dm, _, joint_id, _, _ = anomaly_pars
            joint = self._jns[joint_id]
            margin = 10
            if settings.anomal_iters + 2 * margin > trajectory.shape[0]:
                anomaly_iters = trajectory.shape[0] - 2 * margin
            else:
                anomaly_iters = settings.anomal_iters
            anomaly_start = np.random.randint(low=margin,
                                              high=trajectory.shape[0] - anomaly_iters)

        """ Set up PID controllers """
        ctrl = dict()
        for jn in self._jns:
            ctrl[jn] = PidController(kpid=kpid[jn], direction=PID_DIRECT)
            ctrl[jn].set_output_limits(minmax=tau_lim[jn])

        """ Do PID control """
        t_elapsed = 0.0
        t_start = rospy.get_time()
        cmd = dict()
        started = False
        while (not rospy.is_shutdown() and
               count < trajectory.shape[0] and
               t_elapsed < settings.run_time):
            q0 = self._limb.joint_angles()
            dq0 = self._limb.joint_velocities()

            if anomaly_pars is not None:
                """ Anomaly execution """
                if count >= anomaly_start and not started:
                    started = True
                    anomaly_start = count
                    print " Inducing anomaly on joint", joint
                    kp_mod = kpid[joint][0] * pm
                    ki_mod = kpid[joint][1] * im
                    kd_mod = kpid[joint][2] * dm
                    ctrl[joint].set_parameters(kp=kp_mod, ki=ki_mod,
                                               kd=kd_mod)
                elif anomaly_start < count < anomaly_start + anomaly_iters:
                    self._pub_anom.publish(data=anomaly_pars)
                elif count == anomaly_start + anomaly_iters:
                    ctrl[joint].set_parameters(kpid=kpid[joint])

            for jn in q0.keys():
                idx = self._jns.index(jn)
                cmd[jn] = ctrl[jn].compute(q0[jn], trajectory[count][idx],
                                           dq0[jn])

            command = [trajectory[count][self._jns.index(jn)]
                       for jn in self._rec_joint.get_header_cfg()[1:]]
            self._pub_cfg_comm.publish(command=command)
            command = [ctrl[jn].generated
                       for jn in self._rec_joint.get_header_efft()[1:]]
            self._pub_efft_gen.publish(command=command)

            self._limb.set_joint_torques(cmd)

            rate.sleep()
            t_elapsed = rospy.get_time() - t_start
            count = int(np.floor(t_elapsed * settings.interpolator_rate))
        if count >= trajectory.shape[0]:
            print " Arrived at desired configuration."
        if t_elapsed >= settings.run_time:
            print " Motion timed out."

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
