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

from collections import Sequence
import numpy as np
import rospy
from tf import transformations

from baxter_data_acquisition.settings import joint_names

from geometry_msgs.msg import (
    Pose,
    PoseStamped
)
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest
)


class PoseConfigDuration(Sequence):
    def __init__(self):
        """ Base class for handling poses, configurations and durations. """
        self._data = np.empty(0)

    def __getitem__(self, item):
        return self._data[item, :]

    def __len__(self):
        return len(self._data)

    @staticmethod
    def load_data(file_name):
        """ Load data written to a txt file, where each row contains a sample,
        and each sample is a comma-separated list of entries.
        Use it to load data written, for instance, with numpy.savetxt
        :param file_name: The path to the file to load the data from.
        :return: A numpy array containing the data from the file.
        """
        print "Loading data from '%s'." % file_name
        try:
            data = np.loadtxt(file_name, delimiter=',')
        except IOError:
            raise
        return data

    def _inverse_kinematics(self, pose, arm):
        """ Inverse kinematics of one Baxter arm.
        Compute valid set of joint angles for a given Cartesian pose using the
        inverse kinematics service.
        :param pose: Desired Cartesian pose [x, y. z, a, b, c].
        :return:
        """
        if not isinstance(pose, list) and not (len(pose) == 6 or
                                               len(pose) == 7):
            raise ValueError("Pose must be a list with 6 or 7 entries!")
        qp = self._list_to_pose_stamped(pose, "base")

        node = "ExternalTools/" + arm + "/PositionKinematicsNode/IKService"
        ik_service = rospy.ServiceProxy(node, SolvePositionIK)
        ik_request = SolvePositionIKRequest()
        ik_request.pose_stamp.append(qp)
        try:
            rospy.wait_for_service(node, 5.0)
            ik_response = ik_service(ik_request)
        except (rospy.ServiceException, rospy.ROSException) as error_message:
            raise Exception("Service request failed: %r" % (error_message,))

        if ik_response.isValid[0]:
            # convert response to joint position control dictionary
            cfg = dict(zip(ik_response.joints[0].name,
                           ik_response.joints[0].position))
            return np.asarray([cfg[jn] for jn in joint_names(arm)])
        else:
            raise ValueError("No valid joint configuration found")

    @staticmethod
    def _list_to_pose(pose_list):
        pose_msg = Pose()
        if len(pose_list) == 7:
            pose_msg.position.x = pose_list[0]
            pose_msg.position.y = pose_list[1]
            pose_msg.position.z = pose_list[2]
            pose_msg.orientation.x = pose_list[3]
            pose_msg.orientation.y = pose_list[4]
            pose_msg.orientation.z = pose_list[5]
            pose_msg.orientation.w = pose_list[6]
        elif len(pose_list) == 6:
            pose_msg.position.x = pose_list[0]
            pose_msg.position.y = pose_list[1]
            pose_msg.position.z = pose_list[2]
            q = transformations.quaternion_from_euler(pose_list[3],
                                                      pose_list[4],
                                                      pose_list[5])
            pose_msg.orientation.x = q[0]
            pose_msg.orientation.y = q[1]
            pose_msg.orientation.z = q[2]
            pose_msg.orientation.w = q[3]
        else:
            raise ValueError("Expected either 6 or 7 elements in list: " +
                             "(x,y,z,r,p,y) or (x,y,z,qx,qy,qz,qw)")
        return pose_msg

    def _list_to_pose_stamped(self, pose_list, target_frame):
        pose_msg = PoseStamped()
        pose_msg.pose = self._list_to_pose(pose_list)
        pose_msg.header.frame_id = target_frame
        pose_msg.header.stamp = rospy.Time.now()
        return pose_msg
