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


class CameraRecorder(object):
    def __init__(self):
        """ Camera recorder class writing color ROS image messages recorded
        with the head camera of the baxter robot into a .avi video file and
        timestamps for each image frame into an accompanying .txt file.
        """
        self._clip = None
        self._sub = None
        self._fp = None

        # TODO: use property properly
        # http://stackoverflow.com/questions/17330160/how-does-the-property-decorator-work
        # http://stackoverflow.com/questions/6618002/python-property-versus-getters-and-setters
        self._camera = ""
        self.camera = '/cameras/head_camera/image'

        self._count = 0
        self._t_start = None

    def __str__(self):
        return rospy.get_caller_id()

    def start(self, outname, fps, imgsize):
        """ Set up the camera recorder with the parameters for the recording
        and subscribe to the callback function of the baxter head camera.
        :param outname: Filename to write the video and text file to, without
            the extension.
        :param fps: Frames per second for video file.
        :param imgsize: Size (width, height) of images to write into video
            file.
        :return: Whether the video- and text file were opened successfully.
        """
        self._count = 0
        self._t_start = rospy.get_time()

        try:
            self._fp = open(outname + '.txt', 'w')
        except IOError as e:
            rospy.logfatal("'%s' Failed to open text file!" % self)
            raise e
        self._fp.write('# timestamps [s]\n')

        self._clip = cv2.VideoWriter(outname + '.avi',
                                     fourcc=cv2.cv.CV_FOURCC('M', 'J', 'P', 'G'),
                                     fps=fps,
                                     frameSize=imgsize,
                                     isColor=True)
        if not self._clip.isOpened():
            rospy.logfatal("'%s' Failed to open videoWriter instance!" % self)
            raise IOError("'%s' Failed to open videoWriter instance!" % self)

        # TODO: make sure to use the right camera property here
        self._sub = rospy.Subscriber(self.camera,
                                     Image, callback=self._add_image)
        return self._clip.isOpened() and not self._fp.closed

    def _add_image(self, imgmsg):
        """ Camera subscriber callback function """
        ts = rospy.get_time()
        self._fp.write('%f\n' % ts)
        self._fp.flush()

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
        self._count += 1

    def stop(self):
        """ Stop recording data from the head camera.
        :return: Whether the video- and text file are open.
        """
        # TODO: is there a way to process the remaining subscriber queue before closing it?
        if self._sub is not None:
            rospy.loginfo("'%s' Unregister subscriber ..." % self)
            self._sub.unregister()
            rospy.loginfo("'%s' ... unregistered subscriber." % self)
        rospy.loginfo("'%s' Closing video file ..." % self)
        self._clip.release()
        rospy.loginfo("'%s' ... closed video file." % self)
        rospy.loginfo("'%s' Closing text file ..." % self)
        self._fp.close()
        rospy.loginfo("'%s' ... closed text file." % self)

        self._display_performance()
        return self._clip.isOpened() or self._fp.closed

    def _display_performance(self):
        """ Log performance information (messages received). """
        duration = rospy.get_time() - self._t_start
        rospy.loginfo("'%s' Received %d messages in %.2f s (%.2f Hz)." %
                      (self, self._count, duration, self._count/duration))

    @property
    def camera(self):
        """ String identifying the camera to record images from.
        :return: Camera name.
        """
        return self._camera

    @camera.setter
    def camera(self, camera):
        self._camera = camera


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
