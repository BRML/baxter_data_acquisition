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

import os
import rospy
import signal
import subprocess

from baxter_data_acquisition.srv import Trigger, TriggerResponse


class BagServer(object):
    def __init__(self):
        default_topic = '<ROS_topic_1 ROS_topic_2 ...>'
        topics = rospy.get_param('~topics', default_topic)
        if topics == default_topic:
            rospy.logfatal("Topic '~topics' has not been remapped! Typical command-line usage:")
            rospy.logfatal("  $ rosrun baxter_data_acquisition bag_server _topics:=%s" % default_topic)
            rospy.signal_shutdown("Topic '~topics' is required!")

        self._topics = [t for t in topics.split(' ') if len(t.strip()) > 0]
        rospy.loginfo("Got %d topic(s):" % len(self._topics))
        for idx, topic in enumerate(self._topics):
            rospy.loginfo("- topic %d: %s" % (idx, topic))

        self._running = False
        self._rosbag = None

    def __str__(self):
        return rospy.get_caller_id()

    def clean_shutdown(self):
        rospy.loginfo("Shutting down '%s' ..." % self)
        return True

    def handle_trigger(self, req):
        if req.on and not self._running:
            msg = "Start recording on '%s' ..." % self
            rospy.loginfo(msg)

            start_cmd = "rosbag record"
            if req.outname:
                start_cmd += ' -O %s' % req.outname
            else:
                rospy.logdebug("No filename provided! Recording to default name.")
            for topic in self._topics:
                start_cmd += " %s" % topic
            self._rosbag = subprocess.Popen(start_cmd, shell=True, preexec_fn=os.setsid)

            resp = True if self._rosbag.pid else False
            self._running = True
        elif not req.on and self._running:
            msg = "Stop recording on '%s' ..." % self
            rospy.loginfo(msg)

            os.killpg(self._rosbag.pid, signal.SIGINT)

            resp = True if self._rosbag.pid else False
            self._running = False
        elif req.on and self._running:
            msg = "Recorder is already running. Do nothing."
            rospy.loginfo(msg)
            resp = True
        elif not req.on and not self._running:
            msg = "Recorder is not running. Do nothing."
            rospy.loginfo(msg)
            resp = True
        else:
            msg = 'This cannot happen!'
            rospy.logerr(msg)
            resp = False
        return TriggerResponse(success=resp, message=msg)


if __name__ == "__main__":
    """ A server node for dumping data from ROS topics into a ROS bag file. """
    service_name = 'bag_service'

    rospy.init_node(service_name, anonymous=True, log_level=rospy.INFO)
    bs = BagServer()
    rospy.on_shutdown(bs.clean_shutdown)

    s = rospy.Service(service_name, Trigger, bs.handle_trigger)
    rospy.loginfo("Bag recorder '%s' ready to get triggered ..." % bs)

    rospy.spin()
