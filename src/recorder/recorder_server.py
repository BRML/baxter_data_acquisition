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

from baxter_data_acquisition.srv import TriggerResponse


class RecorderServer(object):
    def __init__(self, recorder):
        """ A skeleton implementation of a recorder server.
        :param recorder: A recorder class object.
        """
        self._rec = recorder()
        self._running = False

    def __str__(self):
        return rospy.get_caller_id()

    def clean_shutdown(self):
        rospy.loginfo("Exiting '%s'." % self)
        self._rec.clean_shutdown()

    def handle_trigger(self, req):
        """ Trigger handle for the recorder server.
        Gets a request service message, specified by Trigger.srv, to start or
        stop the recorder and returns a boolean response indicating the
        successful triggering of the service as well as a descriptive message.
        :param req: A Trigger request service message.
        :returns: A TriggerResponse(bool success, string message).
        """
        if req.on and not self._running:
            # rospy.loginfo("'%s' Starting ..." % self)
            self._running = True
            resp = self._rec.start(outname=req.outname)
            msg = "'%s' ... started." % self
        elif not req.on and self._running:
            # rospy.loginfo("'%s' Stopping ..." % self)
            self._running = False
            resp = self._rec.stop()
            msg = "'%s' ... stopped." % self
        else:
            resp = False
            msg = "'%s' already/not yet running." % self
        # rospy.loginfo(msg)
        return TriggerResponse(success=resp, message=msg)
