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

import rospy

from baxter_data_acquisition.srv import Trigger


class RecorderClient(object):
    def __init__(self, node_namespace=''):
        """ A flash, kinect or senz3d recorder client.
        :param node_namespace: The namespace the server node is pushed into.
        """
        self._ns = node_namespace
        self._service_name = '%s/trigger_service' % self._ns

    def __str__(self):
        return self._ns.replace('_', ' ')

    def start(self, outname):
        """ Start the recorder hosted on the corresponding recorder server.
        :param outname: Filename to write the recorded data to, without the
            extension.
        :return: (bool success, string message)
        """
        rospy.logdebug("Waiting for %s server." % self)
        rospy.wait_for_service(self._service_name)
        try:
            trigger = rospy.ServiceProxy(self._service_name, Trigger)
            resp = trigger(on=True, outname=outname)
            return resp.success, resp.message
        except rospy.ServiceException as e:
            rospy.logerr('%s service call failed: %s' % (self, e))

    def stop(self):
        """ Stop the recorder hosted on the corresponding recorder server.
        :return: (bool success, string message)
        """
        rospy.wait_for_service(self._service_name)
        try:
            trigger = rospy.ServiceProxy(self._service_name, Trigger)
            resp = trigger(on=False)
            return resp.success, resp.message
        except rospy.ServiceException as e:
            rospy.logerr('%s service call failed: %s' % (self, e))
