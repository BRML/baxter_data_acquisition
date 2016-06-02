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


class QueueSubscriber(object):
    def __init__(self, topic, msg_type, callback):
        """ A ROS subscriber augmented with a FIFO queue for messages.

        A queue subscriber starts a ROS subscriber on a given topic and
        message type and puts the received ROS messages in a FIFO queue.
        Upon receiving the stop command, the queue subscriber un-subscribes
        from the topic and processes the messages in the queue with a given
        callback function. Once the queue is empty, the queue subscriber
        returns.
        :param topic: The ROS topic to subscribe to.
        :param msg_type: The ROS message type to use.
        :param callback: The processing function to apply to the received
            ROS messages.
        """
        self._topic = topic
        self._msg_type = msg_type
        self._callback = callback
        # A FIFO queue of infinite size
        self._queue = Queue.Queue(maxsize=0)
        # The ROS subscriber
        self._sub = None
        # Variables for performance measuring
        self._count = 0
        self._t_start = None
        self._t_duration = None

        self._run()

    def __str__(self):
        return "QueueSubscriber on {}".format(rospy.get_caller_id())

    def clean_shutdown(self):
        """ Clean shutdown of the queue subscriber. """
        rospy.loginfo("'%s' Initiate shutdown ..." % self)
        self._ros_unsubscribe()
        self._queue = None
        rospy.loginfo("'%s' ... shutdown completed." % self)

    def _ros_unsubscribe(self):
        """ If we have subscribed to a ROS topic, un-subscribe from it. """
        if self._sub:
            rospy.loginfo("'%s' Un-register ROS subscriber ..." % self)
            self._sub.unregister()
            self._sub = None
            rospy.loginfo("'%s' ... unregistered ROS subscriber." % self)

    def _run(self):
        """ If not yet running, start the queue subscriber by subscribing to
        the ROS topic.
        :raise: RuntimeError if queue subscriber is already running.
        """
        if not self._sub:
            rospy.loginfo("'%s' Register ROS subscriber ..." % self)
            self._count = 0
            self._t_start = rospy.get_time()
            self._sub = rospy.Subscriber(self._topic, self._msg_type,
                                         callback=self._ros_callback,
                                         queue_size=100)
            rospy.loginfo("'%s' ... registered ROS subscriber." % self)
        else:
            raise RuntimeError('QueueSubscriber can only be started once!')

    def _ros_callback(self, msg):
        """ Callback for the ROS subscriber.
        Puts received messages and the current time stamp into a FIFO queue.
        """
        if self._queue:
            self._queue.put((rospy.get_time(), msg))

    def stop(self):
        """ If running, stop the queue recorder by un-subscribing the ROS
        subscriber and processing the messages in the queue.
        :return: Whether the thread (i.e., the instance) is alive.
        :raise: RuntimeError if queue subscriber is not yet running (thread
            has not been started).
        """
        if not rospy.is_shutdown():
            self._ros_unsubscribe()
            if self._t_start:
                self._t_duration = rospy.get_time() - self._t_start
            self._process_msgs()
            self._display_performance()
        return False

    def _process_msgs(self):
        """ Process the messages in the queue with the given callback function. """
        if self._queue.qsize() > 0:
            rospy.loginfo("'%s' Process data queue ..." % self)
            while not rospy.is_shutdown() and not self._queue.empty():
                # Raises Queue.Empty if no item in queue. This should be
                # prevented by checking self._queue.empty() above.
                next_stamped_msg = self._queue.get(block=False)
                self._callback(next_stamped_msg)
                self._queue.task_done()
                self._count += 1
            rospy.loginfo("'%s' ... processed data queue." % self)

    def _display_performance(self):
        """ Log performance information (messages received). """
        if self._t_duration:
            rospy.loginfo("'%s' Received %d messages in %.2f s." %
                          (self, self._count, self._t_duration))
