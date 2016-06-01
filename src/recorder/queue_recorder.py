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

import Queue
import rospy
from threading import Thread


class QueueRecorder(Thread):
    def __init__(self, topic, msg_type, callback):
        super(QueueRecorder, self).__init__()
        self._topic = topic
        self._msg_type = msg_type
        self._callback = callback
        # A FIFO queue of infinite size
        self._queue = Queue.Queue(maxsize=0)
        self._sub = None
        self._count = 0
        self._t_start = None

    def __str__(self):
        return rospy.get_caller_id()

    def clean_shutdown(self):
        rospy.loginfo("'%s' Initiating shutdown ..." % self)
        self._ros_unsubscribe()
        self._queue = None
        rospy.loginfo("'%s' ... shutdown completed." % self)

    def _ros_unsubscribe(self):
        if self._sub is not None:
            rospy.loginfo("'%s' Unregister subscriber ..." % self)
            self._sub.unregister()
            self._sub = None
            rospy.loginfo("'%s' ... unregistered subscriber." % self)

    def run(self):
        if not self._sub:
            rospy.loginfo("'%s' Start QueueRecorder." % self)
            self._count = 0
            self._t_start = rospy.get_time()
            self._sub = rospy.Subscriber(self._topic, self._msg_type,
                                         callback=self._ros_callback,
                                         queue_size=100)
            return True
        else:
            rospy.loginfo("'%s' QueueRecorder already running.")
            return False

    def _ros_callback(self, msg):
        if self._queue:
            self._queue.put(msg)

    def stop(self):
        self._ros_unsubscribe()
        rospy.loginfo("'%s' Process data queue ..." % self)
        while not self._queue.empty():
            next_msg = self._queue.get(block=False)  # Raises Queue.Empty if no item in queue
            self._callback(next_msg)
            self._queue.task_done()
            self._count += 1
        rospy.loginfo("'%s' ... processed data queue." % self)
        self._display_performance()
        self.join(5.0)
        return self.is_alive()

    def _display_performance(self):
        """ Log performance information (messages received). """
        rospy.loginfo("'%s' Received %d messages in %.2f s." %
                      (self, self._count, rospy.get_time() - self._t_start))
