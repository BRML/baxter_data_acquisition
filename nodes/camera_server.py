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
    CameraTrigger,
    CameraTriggerResponse
)
from recorder import CameraRecorder


class Handler(object):
    def __init__(self):
        self._cr = CameraRecorder()
        self._running = False

    def __str__(self):
        return rospy.get_caller_id()

    def clean_shutdown(self):
        rospy.loginfo("Exiting '%s'." % self)
        self._cr.clean_shutdown()

    def handle_trigger(self, req):
        """ Handler handle for the camera recorder server.
        Gets a request service message, specified by Trigger.srv, to start or
        stop the baxter head camera recorder and returns a boolean response
        indicating the successful triggering of the service as well as a
        descriptive message.
        :param req: A Trigger request service message.
        :returns: A TriggerResponse(bool success, string message).
        """
        if req.on and not self._running:
            rospy.loginfo("Starting %s ..." % self)
            self._running = True
            resp = self._cr.start(outname=req.outname,
                                  fps=req.fps,
                                  imgsize=req.size)
            msg = "... %s started." % self
        elif not req.on and self._running:
            rospy.loginfo("Stopping %s ..." % self)
            self._running = False
            resp = self._cr.stop()
            msg = "... %s stopped." % self
        else:
            resp = False
            msg = "%s already/not yet running." % self
        rospy.loginfo(msg)
        return CameraTriggerResponse(success=resp, message=msg)


if __name__ == "__main__":
    """ A server for recording time-stamped videos from baxter's head camera.
    A ROS service server hosting a CameraRecorder instance in a ROS node.
    """
    service_name = 'trigger_service'

    rospy.init_node(service_name, log_level=rospy.INFO)
    h = Handler()
    rospy.on_shutdown(h.clean_shutdown)

    s = rospy.Service(service_name, CameraTrigger, h.handle_trigger)
    rospy.loginfo('%s ready to get triggered.' % h)

    rospy.spin()
