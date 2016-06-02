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

from baxter_data_acquisition.srv import Trigger
from recorder import RecorderServer


class Handler(RecorderServer):
    def __init__(self):
        try:
            recorder_type = rospy.get_param('~type')
        except KeyError:
            rospy.logfatal("ROS parameter '~type' must be set! Typical usage from command line:")
            rospy.logfatal("  $ rosrun baxter_data_acquisition trigger_server.py _type:=<recorder_type>")
            raise KeyError("ROS parameter '~type' must be set!")
        if recorder_type == 'flash':
            from recorder import FlashRecorder as Recorder
        elif recorder_type == 'kinect':
            from recorder import KinectRecorder as Recorder
        elif recorder_type == 'senz3d':
            from recorder import SenzRecorder as Recorder
        else:
            raise ValueError("Recorder type '%s' not implemented!" % recorder_type)
        super(Handler, self).__init__(Recorder)


if __name__ == "__main__":
    """ A ROS server hosting a specific recorder instance in a ROS node. """
    service_name = 'trigger_service'

    rospy.init_node(service_name, log_level=rospy.INFO)
    h = Handler()
    rospy.on_shutdown(h.clean_shutdown)

    s = rospy.Service(service_name, Trigger, h.handle_trigger)
    rospy.loginfo("'%s' ready to get triggered." % h)

    rospy.spin()
