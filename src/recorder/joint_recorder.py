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

import h5py
import numpy as np
import os
import re

import rospy

from baxter_core_msgs.msg import (
    EndpointState,
    JointCommand
)
from sensor_msgs.msg import (
    Imu,
    JointState
)
from std_msgs.msg import (
    UInt16,
    Float64MultiArray
)

from baxter_data_acquisition.srv import JointTrigger


class JointRecorder(object):
    def __init__(self, limb, rate, anomaly_mode='none'):
        """ Joint recorder class writing baxter joint data into a .hdf5 file,
        where the index gives the sample number, containing modalities
        ('configuration', 'effort', 'anomaly', 'acceleration') and their
        corresponding fields ('measured', 'commanded'), if appropriate.
        :param limb: The limb to record data from ['left', 'right'].
        :param rate: The desired recording rate.
        :param anomaly_mode: Type of anomaly in the data ['manual',
        'automatic', 'none'].
        :return: Joint recorder instance.
        """
        self._header = dict()
        self._header['configuration'] = ['time',
                                         limb + '_s0', limb + '_s1',
                                         limb + '_e0', limb + '_e1',
                                         limb + '_w0', limb + '_w1',
                                         limb + '_w2']
        self._header['acceleration'] = ['time', limb + '_x',
                                        limb + '_y', limb + '_z']
        self._header['effort'] = self._header['configuration']
        self._header['pose'] = ['time',
                                limb + '_px', limb + '_py', limb + '_pz',
                                limb + '_qx', limb + '_qy', limb + '_qz',
                                limb + '_qw']
        if anomaly_mode == 'manual':
            self._header['anomaly'] = ['time',
                                       "partId (0-shoulder, 1-s1, "
                                       "2-upper arm, 3-e1, 4-lower arm, "
                                       "5-w1, 6-palm, 7-w2)"]
        elif anomaly_mode == 'automatic':
            self._header['anomaly'] = ['time', 'P multiplier',
                                       'I multiplier', 'D multiplier',
                                       'additive', 'jointId', 'mode', 'type']
        else:
            self._header['anomaly'] = ['', ]
        self._arm = limb
        self._rate = rospy.Rate(rate)
        self._anomaly_mode = anomaly_mode
        self._filename = None
        self._data = None
        self._sub_acc = None
        self._sub_anom = None
        self._sub_cfg_comm = None
        self._sub_cfg_des = None
        self._sub_state = None
        self._sub_efft_comm = None
        self._sub_efft_gen = None
        self._sub_efft_des = None
        self._sub_pose = None
        self._sub_pose_label = None

    def start(self, outfile):
        """ Start joint data recording.
        :param outfile: Filename to write the data to, without the extension.
        """
        self._filename = outfile + '.h5'
        self._data = dict()
        self._data['acceleration'] = dict()
        self._data['acceleration']['measured'] = list()
        self._data['anomaly'] = dict()
        self._data['anomaly']['commanded'] = list()
        self._data['configuration'] = dict()
        self._data['configuration']['commanded'] = list()
        self._data['configuration']['desired'] = list()
        self._data['configuration']['measured'] = list()
        self._data['effort'] = dict()
        self._data['effort']['commanded'] = list()
        self._data['effort']['generated'] = list()
        self._data['effort']['desired'] = list()
        self._data['effort']['measured'] = list()
        self._data['pose'] = dict()
        self._data['pose']['measured'] = list()
        self._data['pose']['label'] = list()

        self._sub_acc = rospy.Subscriber('/robot/accelerometer/' + self._arm +
                                         '_accelerometer/state', Imu,
                                         self._cb_acc, queue_size=1)
        ns = 'data/limb/' + self._arm
        self._sub_anom = rospy.Subscriber(ns + '/anomaly', Float64MultiArray,
                                          self._cb_anom, queue_size=1)
        self._sub_cfg_comm = rospy.Subscriber(ns + '/cfg/comm', JointCommand,
                                              self._cb_cfg_comm, queue_size=1)
        self._sub_cfg_des = rospy.Subscriber(ns + '/cfg/des', JointCommand,
                                             self._cb_cfg_des, queue_size=1)
        self._sub_state = rospy.Subscriber('/robot/joint_states', JointState,
                                           self._cb_state, queue_size=1)
        self._sub_efft_comm = rospy.Subscriber('/robot/limb/' + self._arm +
                                               '/joint_command', JointCommand,
                                               self._cb_efft_comm,
                                               queue_size=1)
        self._sub_efft_gen = rospy.Subscriber(ns + '/efft/gen', JointCommand,
                                              self._cb_efft_gen, queue_size=1)
        self._sub_efft_des = rospy.Subscriber(ns + '/efft/des', JointCommand,
                                              self._cb_efft_des, queue_size=1)
        self._sub_pose = rospy.Subscriber('/robot/limb/' + self._arm +
                                          '/endpoint_state', EndpointState,
                                          self._cb_pose, queue_size=1)
        self._sub_pose_label = rospy.Subscriber(ns + '/pose/label', UInt16,
                                                self._cb_pose_label,
                                                queue_size=1)

    def stop(self):
        """ Stop joint data recording. """
        if self._sub_acc is not None:
            rospy.loginfo('unregistering acceleration ...')
            self._sub_acc.unregister()
        if self._sub_anom is not None:
            rospy.loginfo('unregistering anomalies...')
            self._sub_anom.unregister()
        if self._sub_cfg_comm is not None:
            rospy.loginfo('unregistering config commanded ...')
            self._sub_cfg_comm.unregister()
        if self._sub_cfg_des is not None:
            rospy.loginfo('unregistering config desired ...')
            self._sub_cfg_des.unregister()
        if self._sub_state is not None:
            rospy.loginfo('unregistering robot state ...')
            self._sub_state.unregister()
        if self._sub_efft_comm is not None:
            rospy.loginfo('unregistering effort commanded ...')
            self._sub_efft_comm.unregister()
        if self._sub_efft_gen is not None:
            rospy.loginfo('unregistering effort generated ...')
            self._sub_efft_gen.unregister()
        if self._sub_efft_des is not None:
            rospy.loginfo('unregistering effort desired ...')
            self._sub_efft_des.unregister()
        if self._sub_pose is not None:
            rospy.loginfo('unregistering pose ...')
            self._sub_pose.unregister()
        if self._sub_pose_label is not None:
            rospy.loginfo('unregistering pose label ...')
            self._sub_pose_label.unregister()

    def write_sample(self):
        """ Append data of one sample to the .hdf5 joint data file. """
        if not os.path.exists(self._filename):
            mode = 'w'
        else:
            mode = 'a'
        with h5py.File(self._filename, mode) as fp:
            idx = len(fp)
            g = fp.require_group('%i' % idx)
            for modality in self._data:
                if modality == 'anomaly':
                    if len(self._data['anomaly']['commanded']) == 0:
                        # no anomaly in this sample
                        continue
                elif modality == 'acceleration':
                    if len(self._data['acceleration']['measured']) == 0:
                        # no acceleration in this sample (simulation)
                        continue
                gm = g.require_group(modality)
                for field in self._data[modality]:
                    data = np.asarray(self._data[modality][field])
                    if data.shape == (0,):
                        continue
                    gf = gm.require_dataset(field, shape=data.shape,
                                            dtype=data.dtype, data=data)
                    if modality == 'pose' and field == 'label':
                        gf.attrs.create('Header', data=['time', 'index'])
                    else:
                        gf.attrs.create('Header', data=self._header[modality])
        # print 'Done writing sample data to file.'

    def _cb_acc(self, data):
        acc = data.linear_acceleration
        self._data['acceleration']['measured'].append([rospy.get_time(),
                                                       acc.x, acc.y, acc.z])
        self._rate.sleep()

    def _cb_anom(self, data):
        anom = list(data.data)
        self._data['anomaly']['commanded'].append([rospy.get_time()] + anom)

    def _cb_cfg_comm(self, data):
        cfg = list(data.command)
        self._data['configuration']['commanded'].append(
                [rospy.get_time()] + cfg)
        self._rate.sleep()

    def _cb_cfg_des(self, data):
        cfg = list(data.command)
        self._data['configuration']['desired'].append([rospy.get_time()] + cfg)

    def _cb_state(self, data):
        time = rospy.get_time()
        name = list(data.name)
        pos = list(data.position)
        effort = list(data.effort)
        try:
            p = [pos[name.index(j)] for j in self._header['configuration'][1:]]
            self._data['configuration']['measured'].append([time] + p)
            e = [effort[name.index(j)] for j in self._header['effort'][1:]]
            self._data['effort']['measured'].append([time] + e)
        except ValueError:
            # there seems to be a BUG: every few callbacks name is not joints
            # but ['r_gripper_l_finger_joint'] with corresponding pos and efft
            pass
        self._rate.sleep()

    def _cb_efft_comm(self, data):
        name = list(data.names)
        effort = list(data.command)
        try:
            e = [effort[name.index(j)] for j in self._header['effort'][1:]]
            self._data['effort']['commanded'].append([rospy.get_time()] + e)
        except ValueError:
            print "ERROR-cb_efft_comm %i-Key does not exist." % data.header.seq
        self._rate.sleep()

    def _cb_efft_gen(self, data):
        effort = list(data.command)
        self._data['effort']['generated'].append([rospy.get_time()] + effort)
        self._rate.sleep()

    def _cb_efft_des(self, data):
        effort = list(data.command)
        self._data['effort']['desired'].append([rospy.get_time()] + effort)
        self._rate.sleep()

    def _cb_pose(self, data):
        pose = [data.pose.position.x,
                data.pose.position.y,
                data.pose.position.z,
                data.pose.orientation.x,
                data.pose.orientation.y,
                data.pose.orientation.z,
                data.pose.orientation.w]
        self._data['pose']['measured'].append([rospy.get_time()] + pose)
        self._rate.sleep()

    def _cb_pose_label(self, data):
        self._data['pose']['label'].append([rospy.get_time(), data.data])
        self._rate.sleep()

    def get_header_acc(self):
        """ Return acceleration data header.
        :return: Acceleration data header.
        """
        return self._header['acceleration']

    def get_header_anom(self):
        """ Return anomaly data header.
        :return: Anomaly data header.
        """
        if self._anomaly_mode == 'manual':
            header = self._header['anomaly']
            header[1] = re.sub('\(.*?\)', '', header[1]).strip()
            return header
        return self._header['anomaly']

    def get_header_cfg(self):
        """ Return configuration data header.
        :return: Configuration data header.
        """
        return self._header['configuration']

    def get_header_efft(self):
        """ Return effort data header.
        :return: Effort data header.
        """
        return self._header['effort']

    def get_header_pose(self):
        """ Return pose data header.
        :return: Pose data header.
        """
        return self._header['pose']


class JointClient(object):
    def __init__(self, limb, rate, anomaly_mode='none'):
        """ Joint recorder class writing baxter joint data into a .hdf5 file,
        where the index gives the sample number, containing modalities
        ('configuration', 'effort', 'anomaly', 'acceleration') and their
        corresponding fields ('measured', 'commanded'), if appropriate.
        :param limb: The limb to record data from ['left', 'right'].
        :param rate: The desired recording rate.
        :param anomaly_mode: Type of anomaly in the data ['manual',
        'automatic', 'none'].
        """
        self._service_name = 'joint_service'

        # set up JointRecorder instance on server node
        rospy.loginfo("Waiting for joint recorder server.")
        rospy.wait_for_service(self._service_name)
        try:
            trigger = rospy.ServiceProxy(self._service_name, JointTrigger)
            resp = trigger(setup='%s, %s, %s' % (limb, rate, anomaly_mode))
            if not resp.success:
                s = "Unable to set up JointRecorder instance!"
                rospy.logfatal(s)
                raise ValueError(s)
            rospy.loginfo(resp.message)
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s' % e)

    def start(self, outfile):
        """ Start joint recorder hosted on joint recorder server.
        :param outfile: Filename to write the data to, without the extension.
        :return: (bool success, string message)
        """
        rospy.wait_for_service(self._service_name)
        try:
            trigger = rospy.ServiceProxy(self._service_name, JointTrigger)
            resp = trigger(task='on', outname=outfile)
            return resp.success, resp.message
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s' % e)

    def stop(self):
        """ Stop joint recorder hosted on joint recorder server.
        :return: (bool success, string message)
        """
        rospy.wait_for_service(self._service_name)
        try:
            trigger = rospy.ServiceProxy(self._service_name, JointTrigger)
            resp = trigger(task='off')
            return resp.success, resp.message
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s' % e)

    def write_sample(self):
        """ Append data of one sample to the .hdf5 joint data file. """
        rospy.wait_for_service(self._service_name)
        try:
            trigger = rospy.ServiceProxy(self._service_name, JointTrigger)
            resp = trigger(task='write')
            return resp.success, resp.message
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s' % e)

    def get_header_acc(self):
        """ Return acceleration data header.
        :return: The acceleration header.
        """
        rospy.wait_for_service(self._service_name)
        try:
            trigger = rospy.ServiceProxy(self._service_name, JointTrigger)
            resp = trigger(modality='acceleration')
            return resp.header
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s' % e)

    def get_header_anom(self):
        """ Return anomaly data header.
        :return: The anomaly header.
        """
        rospy.wait_for_service(self._service_name)
        try:
            trigger = rospy.ServiceProxy(self._service_name, JointTrigger)
            resp = trigger(modality='anomaly')
            return resp.header
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s' % e)

    def get_header_cfg(self):
        """ Return configuration data header.
        :return: The configuration header.
        """
        rospy.wait_for_service(self._service_name)
        try:
            trigger = rospy.ServiceProxy(self._service_name, JointTrigger)
            resp = trigger(modality='configuration')
            return resp.header
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s' % e)

    def get_header_efft(self):
        """ Return effort data header.
        :return: The effort header.
        """
        rospy.wait_for_service(self._service_name)
        try:
            trigger = rospy.ServiceProxy(self._service_name, JointTrigger)
            resp = trigger(modality='effort')
            return resp.header
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s' % e)

    def get_header_pose(self):
        """ Return pose data header.
        :return: The pose header.
        """
        rospy.wait_for_service(self._service_name)
        try:
            trigger = rospy.ServiceProxy(self._service_name, JointTrigger)
            resp = trigger(modality='pose')
            return resp.header
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s' % e)
