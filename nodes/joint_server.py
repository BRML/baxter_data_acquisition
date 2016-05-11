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
    JointTrigger,
    JointTriggerResponse
)
from recorder import JointRecorder


class Handler(object):
    def __init__(self):
        self._jr = None
        self._setup = False
        self._running = False

    def handle_trigger(self, req):
        """ Handler handle for the joint recorder server.
        Gets a request service message, specified by Trigger.srv, to start or
        stop the joint recorder and returns a boolean response indicating
        the successful triggering of the service as well as a descriptive
        message. Can additionally return a list of header labels.
        :param req: A Trigger request service message.
        :returns: A TriggerResponse(bool success, string message).
        """
        header = ''
        if not self._setup:
            if req.setup != '':
                limb, rate, mode = [x.strip() for x in req.setup.split(',')]
                self._jr = JointRecorder(limb=limb, rate=float(rate),
                                         anomaly_mode=mode)
                self._setup = True
                resp = True
                msg = "Joint recorder successfully set up."
            else:
                msg = "Need to set up JointClient first!"
                rospy.logerr(msg)
                return JointTriggerResponse(success=False, message=msg)
        else:  # Joint Recorder is set up, joint server can do its thing
            if req.setup != '':
                resp = True
                msg = "Joint recorder already set up."
            elif req.task == 'on':
                if not self._running:
                    rospy.loginfo("Starting joint recorder ...")
                    self._jr.start(outfile=req.outname)
                    self._running = True
                    resp = True
                    msg = "Started joint recorder."
                else:
                    resp = False
                    msg = "Joint recorder already running."
            elif req.task == 'off':
                if self._running:
                    rospy.loginfo("Stopping joint recorder ...")
                    self._jr.stop()
                    self._running = False
                    resp = True
                    msg = "Stopped joint recorder."
                else:
                    resp = False
                    msg = "Joint recorder not yet running."
            elif req.task == 'write':
                if not self._running:
                    rospy.loginfo("Writing joint data ...")
                    self._jr.write_sample()
                    resp = True
                    msg = "Done writing joint data for sample to file."
                else:
                    resp = False
                    msg = "Need to stop joint recorder first."
            else:
                if req.modality != '':
                    resp = True
                    rospy.logdebug("Retrieving %s-header." % req.modality)
                    msg = ''
                    if req.modality == 'acceleration':
                        header = self._jr.get_header_acc()
                    elif req.modality == 'anomaly':
                        header = self._jr.get_header_anom()
                    elif req.modality == 'configuration':
                        header = self._jr.get_header_cfg()
                    elif req.modality == 'effort':
                        header = self._jr.get_header_efft()
                    elif req.modality == 'pose':
                        header = self._jr.get_header_pose()
                    else:
                        resp = False
                        msg = "No such header: '%s'." % req.modality
                else:
                    resp = False
                    msg = "No such joint recorder command."
        if msg != '':
            rospy.loginfo(msg)
        return JointTriggerResponse(success=resp, message=msg, header=header)


def main():
    """ A server for recording time-stamped data from baxter's internal
    sensors.
    A ROS service server hosting a JointRecorder instance in a ROS node.
    """
    service_name = 'joint_service'

    rospy.init_node(service_name, log_level=rospy.INFO)
    h = Handler()
    s = rospy.Service(service_name, JointTrigger, h.handle_trigger)
    rospy.loginfo('Joint recorder ready to get triggered.')
    rospy.spin()


if __name__ == "__main__":
    main()
