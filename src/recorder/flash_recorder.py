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

from std_msgs.msg import Float64

from recorder.queue_subscriber import QueueSubscriber


class FlashRecorder(object):
    def __init__(self):
        """ Flash time stamp recorder class writing time stamps for each
        white flash of the robot's head screen into a .txt file.
        """
        self._topic = 'chatter'  # '/data/head/flash_white'
        self._msg_type = Float64
        self._fp = None
        self._sub = None
        self._running = False

    def __str__(self):
        return rospy.get_caller_id()

    def clean_shutdown(self):
        if self._sub:
            self._sub.clean_shutdown()
        self._close_file()
        self._running = False

    def _close_file(self):
        if self._fp:
            rospy.loginfo("'%s' Closing text file ..." % self)
            self._fp.close()
            self._fp = None
            rospy.loginfo("'%s' ... closed text file." % self)

    def start(self, outname):
        if not rospy.is_shutdown():
            if not self._running:
                try:
                    self._fp = open('{}.txt'.format(outname), 'w')
                except IOError as e:
                    rospy.logfatal("'%s' Failed to open text file!" % self)
                    raise e
                self._fp.write('# timestamps [s]\n')
                self._sub = QueueSubscriber(topic=self._topic,
                                            msg_type=self._msg_type,
                                            callback=self._process_msg)
                self._sub.start()
                self._running = True
            else:
                rospy.loginfo("'%s' Already running. I do nothing." % self)
            return not self._fp.closed
        return False

    def stop(self):
        if not rospy.is_shutdown():
            if self._running:
                if self._sub:
                    self._sub.stop()
                    self._sub = None
                self._close_file()
                self._running = False
                return False
            else:
                rospy.loginfo("'%s' Not running. I do nothing." % self)
                return not self._fp.closed
        return False

    def _process_msg(self, msg):
        """ Flash subscriber callback function """
        self._fp.write('%f\n' % msg.data)
        self._fp.flush()


# class FlashRecorder(object):
#     def __init__(self):
#         """ Flash time stamp recorder class writing time stamps for each
#         white flash of the robot's head screen into a .txt file.
#         """
#         self._sub = None
#         self._fp = None
#
#         self._count = 0
#         self._t_start = None
#
#     def __str__(self):
#         return rospy.get_caller_id()
#
#     def start(self, outname):
#         """ Set up the flash recorder with the parameters for the recording
#         and subscribe to the callback function of the screen flash subscriber.
#         :param outname: Filename to write the text file to, without the
#             extension.
#         :return: Whether the text file was opened successfully.
#         """
#         self._count = 0
#         self._t_start = rospy.get_time()
#
#         try:
#             self._fp = open(outname + '.txt', 'w')
#         except IOError as e:
#             rospy.logfatal("'%s' Failed to open text file!" % self)
#             raise e
#         self._fp.write('# timestamps [s]\n')
#
#         self._sub = rospy.Subscriber('/data/head/flash_white',
#                                      Float64, callback=self._add_timestamp,
#                                      queue_size=3)
#         return not self._fp.closed
#
#     def _add_timestamp(self, ts):
#         """ Flash subscriber callback function """
#         self._fp.write('%f\n' % ts.data)
#         self._fp.flush()
#         self._count += 1
#
#     def stop(self):
#         """ Stop recording head screen flash time stamps.
#         :return: Whether the text file is open.
#         """
#         if self._sub is not None:
#             rospy.loginfo("'%s' Unregister subscriber ..." % self)
#             self._sub.unregister()
#             rospy.loginfo("'%s' ... unregistered subscriber." % self)
#         # rospy.loginfo("'%s' Closing text file ..." % self)
#         # self._fp.close()
#         # rospy.loginfo("'%s' ... closed text file." % self)
#
#         self._display_performance()
#         return self._fp.closed
#
#     def _display_performance(self):
#         """ Log performance information (messages received). """
#         rospy.loginfo("'%s' Received %d messages in %.2f s." %
#                       (self, self._count, rospy.get_time() - self._t_start))
