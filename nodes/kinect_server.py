#!/usr/bin/env python

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

from baxter_data_acquisition.srv import (
    Trigger,
    TriggerResponse
)
from recorder import KinectRecorder


class Handler(object):
    def __init__(self):
        self._kr = KinectRecorder()
        self._running = False

    def handle_trigger(self, req):
        """ Handler handle for the Kinect recorder server.
        Gets a request service message, specified by Trigger.srv, to start or
        stop the Kinect V2 sensor recorder and returns a boolean response
        indicating the successful triggering of the service as well as a
        descriptive message.
        :param req: A Trigger request service message.
        :returns: A TriggerResponse(bool success, string message).
        """
        if req.on and not self._running:
            self._running = True
            resp = self._kr.start(outname=req.outname)
            msg = "Started Kinect recorder."
        elif not req.on and self._running:
            self._running = False
            resp = self._kr.stop()
            msg = "Stopped Kinect recorder."
        else:
            resp = False
            msg = "Kinect recorder already/not yet running."
        rospy.logdebug(msg)
        return TriggerResponse(success=resp, message=msg)


if __name__ == "__main__":
    """ A server for recording time-stamped data from the Kinect V2 sensor.
    A ROS service server hosting a KinectRecorder instance in a ROS node.
    """
    service_name = 'kinect_service'

    rospy.init_node(service_name)
    h = Handler()
    s = rospy.Service(service_name, Trigger, h.handle_trigger)
    rospy.loginfo('Kinect recorder ready to get triggered.')
    rospy.spin()
