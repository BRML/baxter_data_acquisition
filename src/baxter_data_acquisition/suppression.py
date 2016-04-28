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
import threading

from std_msgs.msg import Empty


class Suppressor(threading.Thread):
    def __init__(self, limb, name):
        """ Suppressing collision avoidance or collision detection on one of
        baxter's limbs using a separate thread.
        :param limb: The limb to suppress functionality on.
        :param name: The string identifying the functionality.
        :return: A Suppressor Thread instance.
        """
        super(Suppressor, self).__init__()
        if limb not in ['left', 'right']:
            raise ValueError("'limb' must be 'left' or 'right'!")

        self._stop = False
        self._pub = rospy.Publisher('robot/limb/'+limb+name, Empty,
                                    queue_size=10)

    def run(self):
        """ Send suppress signal at rate > 5 Hz. """
        while not rospy.is_shutdown() and not self._stop:
            self._pub.publish()
            rospy.sleep(0.1)

    def stop(self):
        """ End the thread's while loop sending the suppress signal.
        :return: Whether the tread is alive.
        """
        self._stop = True
        self.join(5.0)
        return self.is_alive()


class AvoidanceSuppressor(Suppressor):
    def __init__(self, limb):
        """ Suppressing collision avoidance on one of baxter's limbs using a
        separate thread.
        :param limb: The limb to suppress collision avoidance on.
        :return: A Suppressor Thread instance.
        """
        name = '/suppress_collision_avoidance'
        super(AvoidanceSuppressor, self).__init__(limb, name)


class DetectionSuppressor(Suppressor):
    def __init__(self, limb):
        """ Suppressing collision detection on one of baxter's limbs using a
        separate thread.
        :param limb: The limb to suppress collision detection on.
        :return: A Suppressor Thread instance.
        """
        name = '/suppress_contact_safety'
        super(DetectionSuppressor, self).__init__(limb, name)
