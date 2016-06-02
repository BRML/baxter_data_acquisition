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
from recorder.queue_subscriber import QueueSubscriber


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
        self._arm = limb
        self._rate = rospy.Rate(rate)
        self._anomaly_mode = anomaly_mode

        self._header = dict()
        self._define_headers()

        self._filename = None
        self._topics = list()
        self._define_subscriber_topics()
        self._data = None
        self._subs = {n: None for n in self._topics}

    def __str__(self):
        return rospy.get_caller_id()

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

        for name in self._topics:
            if rospy.is_shutdown():
                break
            topic, msg_type, callback = self._topics[name]
            self._subs[name] = QueueSubscriber(topic=topic,
                                               msg_type=msg_type,
                                               callback=callback)
        return True

    def stop(self):
        """ Stop joint data recording. """
        for name in self._subs:
            if rospy.is_shutdown():
                break
            if self._subs[name]:
                self._subs[name].stop()
                self._subs[name] = None
        return True

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

    def _define_subscriber_topics(self):
        """ Define the subscriber topics for the different recorded modalities.
        Each entry is  a tuple defining
          {name: (ROS topic, ROS message type, callback function)}.
        """
        ns = '/data/limb/' + self._arm
        self._topics = {'acceleration measured':
                            ('/robot/accelerometer/' + self._arm + '_accelerometer/state',
                             Imu,
                             self._cb_acc),
                        'anomaly commanded':
                            (ns + '/anomaly',
                             Float64MultiArray,
                             self._cb_anom),
                        'configuration commanded':
                            (ns + '/cfg/comm',
                             JointCommand,
                             self._cb_cfg_comm),
                        'configuration desired':
                            (ns + '/cfg/des',
                             JointCommand,
                             self._cb_cfg_des),
                        'configuration/effort measured':
                            ('/robot/joint_states',
                             JointState,
                             self._cb_state),
                        'effort commanded':
                            ('/robot/limb/' + self._arm + '/joint_command',
                             JointCommand,
                             self._cb_efft_comm),
                        'effort generated':
                            (ns + '/efft/gen',
                             JointCommand,
                             self._cb_efft_gen),
                        'effort desired':
                            (ns + '/efft/des',
                             JointCommand,
                             self._cb_efft_des),
                        'pose measured':
                            ('/robot/limb/' + self._arm + '/endpoint_state',
                             EndpointState,
                             self._cb_pose),
                        'pose label':
                            (ns + '/pose/label',
                             UInt16,
                             self._cb_pose_label)}

    def _cb_acc(self, stamped_msg):
        ts, msg = stamped_msg
        acc = msg.linear_acceleration
        self._data['acceleration']['measured'].append([ts,
                                                       acc.x, acc.y, acc.z])
        self._rate.sleep()

    def _cb_anom(self, stamped_msg):
        ts, msg = stamped_msg
        anom = list(msg.data)
        self._data['anomaly']['commanded'].append([ts] + anom)

    def _cb_cfg_comm(self, stamped_msg):
        ts, msg = stamped_msg
        cfg = list(msg.command)
        self._data['configuration']['commanded'].append([ts] + cfg)
        self._rate.sleep()

    def _cb_cfg_des(self, stamped_msg):
        ts, msg = stamped_msg
        cfg = list(msg.command)
        self._data['configuration']['desired'].append([ts] + cfg)

    def _cb_state(self, stamped_msg):
        ts, msg = stamped_msg
        name = list(msg.name)
        pos = list(msg.position)
        effort = list(msg.effort)
        try:
            p = [pos[name.index(j)] for j in self._header['configuration'][1:]]
            self._data['configuration']['measured'].append([ts] + p)
            e = [effort[name.index(j)] for j in self._header['effort'][1:]]
            self._data['effort']['measured'].append([ts] + e)
        except ValueError:
            # there seems to be a BUG: every few callbacks name is not joints
            # but ['r_gripper_l_finger_joint'] with corresponding pos and efft
            pass
        self._rate.sleep()

    def _cb_efft_comm(self, stamped_msg):
        ts, msg = stamped_msg
        name = list(msg.names)
        effort = list(msg.command)
        try:
            e = [effort[name.index(j)] for j in self._header['effort'][1:]]
            self._data['effort']['commanded'].append([ts] + e)
        except ValueError:
            print "ERROR-cb_efft_comm %i-Key does not exist." % msg.header.seq
        self._rate.sleep()

    def _cb_efft_gen(self, stamped_msg):
        ts, msg = stamped_msg
        effort = list(msg.command)
        self._data['effort']['generated'].append([ts] + effort)
        self._rate.sleep()

    def _cb_efft_des(self, stamped_msg):
        ts, msg = stamped_msg
        effort = list(msg.command)
        self._data['effort']['desired'].append([ts] + effort)
        self._rate.sleep()

    def _cb_pose(self, stamped_msg):
        ts, msg = stamped_msg
        pose = [msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w]
        self._data['pose']['measured'].append([ts] + pose)
        self._rate.sleep()

    def _cb_pose_label(self, stamped_msg):
        ts, msg = stamped_msg
        self._data['pose']['label'].append([ts, msg.data])
        self._rate.sleep()

    def _define_headers(self):
        """ Define the headers for the different recorded modalities. """
        self._header['acceleration'] = \
            ['time'] + [self._arm + '_' + a for a in ['x', 'y', 'z']]

        self._header['configuration'] = \
            ['time'] + [self._arm + '_' + j
                        for j in ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]
        self._header['effort'] = self._header['configuration']

        self._header['pose'] = \
            ['time'] + [self._arm + '_' + k
                        for k in ['px', 'py', 'pz', 'qx', 'qy', 'qz', 'qw']]

        if self._anomaly_mode == 'manual':
            self._header['anomaly'] = ['time',
                                       "partId (0-shoulder, 1-s1, "
                                       "2-upper arm, 3-e1, 4-lower arm, "
                                       "5-w1, 6-palm, 7-w2)"]
        elif self._anomaly_mode == 'automatic':
            self._header['anomaly'] = ['time', 'P multiplier',
                                       'I multiplier', 'D multiplier',
                                       'additive', 'jointId', 'mode', 'type']
        else:
            self._header['anomaly'] = ['']

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
        self._service_name = 'joint_recorder/trigger_service'

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
