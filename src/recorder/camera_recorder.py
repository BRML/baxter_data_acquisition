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

import cv2
import cv_bridge

import rospy

from sensor_msgs.msg import Image

from baxter_data_acquisition.srv import CameraTrigger
from recorder.queue_subscriber import QueueSubscriber


class CameraRecorder(object):
    def __init__(self):
        """ Camera recorder class writing color ROS image messages recorded
        with the head camera of the baxter robot into a .avi video file and
        timestamps for each image frame into an accompanying .txt file.
        """
        self._topic = '/cameras/head_camera/image'
        self._msg_type = Image
        self._clip = None
        self._fp = None
        self._sub = None
        self._running = False

    def __str__(self):
        return rospy.get_caller_id()

    def clean_shutdown(self):
        """ Clean shutdown of the camera recorder. """
        if self._sub:
            self._sub.clean_shutdown()
        self._close_clip()
        self._close_file()
        self._running = False

    def _close_clip(self):
        """ Close video file we wrote image frames into. """
        if self._clip:
            rospy.loginfo("'%s' Closing video file ..." % self)
            self._clip.release()
            rospy.loginfo("'%s' ... closed video file." % self)

    def _close_file(self):
        """ Close text file we wrote time stamps into. """
        if self._fp:
            rospy.loginfo("'%s' Closing text file ..." % self)
            self._fp.close()
            self._fp = None
            rospy.loginfo("'%s' ... closed text file." % self)

    def start(self, outname, fps, imgsize):
        """ Set up the camera recorder with the parameters for the recording
        and subscribe to the callback function of the camera.
        :param outname: Filename to write the video and text file to, without
            the extension.
        :param fps: Frames per second for video file.
        :param imgsize: Size (width, height) of images to write into video
            file.
        :return: Whether the video- and text file were opened successfully.
        """
        if not rospy.is_shutdown():
            if not self._running:
                # open text file
                try:
                    self._fp = open(outname + '.txt', 'w')
                except IOError as e:
                    rospy.logfatal("'%s' Failed to open text file!" % self)
                    raise e
                self._fp.write('# timestamps [s]\n')
                # open video file
                self._clip = cv2.VideoWriter(outname + '.avi',
                                             fourcc=cv2.cv.CV_FOURCC('M', 'J', 'P', 'G'),
                                             fps=fps,
                                             frameSize=imgsize,
                                             isColor=True)
                if not self._clip.isOpened():
                    rospy.logfatal("'%s' Failed to open videoWriter instance!" % self)
                    raise IOError("'%s' Failed to open videoWriter instance!" % self)
                # start subscriber
                self._sub = QueueSubscriber(topic=self._topic,
                                            msg_type=self._msg_type,
                                            callback=self._add_image)
                self._running = True
            else:
                rospy.logwarn("'%s' Already running. I do nothing." % self)
            return self._clip.isOpened() and not self._fp.closed
        return False

    def _add_image(self, stamped_msg):
        """ Camera subscriber callback function """
        ts, imgmsg = stamped_msg
        # write time stamp to file
        self._fp.write('%f\n' % ts)
        self._fp.flush()
        # add frame to video
        try:
            img = cv_bridge.CvBridge().imgmsg_to_cv2(imgmsg, 'bgr8')
        except cv_bridge.CvBridgeError as e:
            rospy.logfatal("'%s' Failed to convert ROS image message!" % self)
            raise e
        try:
            self._clip.write(img)
        except Exception as e:
            rospy.logfatal("'%s' Recording frame failed!" % self)
            raise e

    def stop(self):
        """ Stop recording data from the camera.
        :return: Whether the video- and text file are open.
        """
        if not rospy.is_shutdown():
            if self._running:
                if self._sub:
                    self._sub.stop()
                    self._sub = None
                self._close_clip()
                self._close_file()
                self._running = False
                return False
            else:
                rospy.logwarn("'%s' Not running. I do nothing." % self)
                return not self._fp.closed
        return False

    @property
    def topic(self):
        """ Get the camera topic to record images from. """
        return self._topic

    @topic.setter
    def topic(self, topic_name):
        """ Set the camera topic to record images from. """
        self._topic = topic_name

    @topic.deleter
    def topic(self):
        """ Delete the camera topic to record images from. """
        del self._topic


class CameraClient(object):
    def __init__(self):
        self._service_name = 'camera_recorder/trigger_service'

    def start(self, outname, fps, imgsize):
        """ Start camera recorder hosted on camera recorder server.
        :param outname: Filename to write the video and text file to, without
        the extension.
        :param fps: Frames per second for video file.
        :param imgsize: Size (width, height) of images to write into video
        file.
        :return: (bool success, string message)
        """
        rospy.logdebug("Waiting for camera recorder server.")
        rospy.wait_for_service(self._service_name)
        try:
            trigger = rospy.ServiceProxy(self._service_name, CameraTrigger)
            resp = trigger(on=True, outname=outname, fps=fps, size=imgsize)
            return resp.success, resp.message
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s' % e)

    def stop(self):
        """ Stop camera recorder hosted on camera recorder server.
        :return: (bool success, string message)
        """
        rospy.wait_for_service(self._service_name)
        try:
            trigger = rospy.ServiceProxy(self._service_name, CameraTrigger)
            resp = trigger(on=False)
            return resp.success, resp.message
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s' % e)
