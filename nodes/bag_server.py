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

import rosbag
import rospy

from std_srvs.srv import Trigger


class BagServer(object):
    def __init__(self):
        topics = rospy.get_param('~topics', 'topic')
        print topics
        if topics == 'topic':
            rospy.logwarn("Topic '~topics' has not been remapped! Typical command-line usage:")
            rospy.logwarn("  $ rosrun baxter_data_acquisition bag_server _topics:=<ROS topic>")
            rospy.signal_shutdown("Topic '~topics' is required!")
        else:
            self._topics = topics.split(' ')
            for topic in self._topics:
                print "Topic '~topics' is '%s'." % rospy.resolve_name(topic)

        self._started = False

    def clean_shutdown(self):
        rospy.loginfo("Shutting down bag server ...")
        return True

    def handle_trigger(self):
        if self._started:
            rospy.loginfo("Stop recording from {} ...".format(self._topics))
            self._started = False
        else:
            rospy.loginfo("Start recording from {} ...".format(self._topics))
            # Writing stuff to the bag file requires Subscribers again ...
            #   http://wiki.ros.org/rosbag/Code%20API
            # This is exactly what I wanted to avoid ...
            self._started = True


if __name__ == "__main__":
    """ A server node for dumping data from ROS topics into a ROS bag file. """
    service_name = 'bag_service'

    rospy.init_node(service_name, anonymous=True, log_level=rospy.INFO)
    bs = BagServer()
    rospy.on_shutdown(bs.clean_shutdown)

    s = rospy.Service(service_name, Trigger, bs.handle_trigger)
    rospy.loginfo('Bag recorder ready to get triggered ...')

    rospy.spin()
