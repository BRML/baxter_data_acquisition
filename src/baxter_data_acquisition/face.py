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
import numpy as np
import rospy

from sensor_msgs.msg import Image
from std_msgs.msg import Float64


def send_image(path):
    """ Display the specified image on baxter's head display.
    :param path: Path to the image file to load and display on baxter.
    """
    imgmsg = img2imgmsg(cv2.imread(path))
    send_imgmsg(imgmsg)


def send_imgmsg(imgmsg):
    """ Display a ROS image message on baxter's head display.
    :param imgmsg: a ROS image message.
    """
    pub = rospy.Publisher('/robot/xdisplay', Image, queue_size=10, latch=True)
    try:
        pub.publish(imgmsg)
    except TypeError:
        raise TypeError('Something is wrong with the ROS image message!')


def flash_screen(repetitions, duration1, duration2):
    """ Flash baxter's head display a number of times.
    :param repetitions: Number of flashes.
    :param duration1: Duration [s] display is dark during one repetition.
    :param duration2: Duration [s] display is bright during one repetition.
    :return: True on completion
    """
    imgmsg_black = black_imgmsg()
    imgmsg_white = white_imgmsg()
    r = True
    pub = rospy.Publisher('data/head/flash_white', Float64,
                          queue_size=10, latch=True)

    send_imgmsg(imgmsg_black)
    rospy.sleep(duration1 + duration2)
    for i in range(2*repetitions):
        if r:
            send_imgmsg(imgmsg_black)
            rospy.sleep(duration1)
        else:
            send_imgmsg(imgmsg_white)
            pub.publish(data=rospy.get_time())
            rospy.sleep(duration2)
        r = not r
    send_imgmsg(imgmsg_black)
    return True


def white_imgmsg():
    """ A white image of size 1024x600 pixels fitting the screen of the baxter
    robot.
    :return: a ROS image message.
    """
    img = np.ones((600, 1024, 1), dtype=np.uint8)
    img *= 255
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    return img2imgmsg(img)


def black_imgmsg():
    """ A black image of size 1024x600 pixels fitting the screen of the baxter
    robot.
    :return: a ROS image message.
    """
    img = np.zeros((600, 1024, 1), dtype=np.uint8)
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    return img2imgmsg(img)


def img2imgmsg(img):
    """ Convert a numpy array holding an image to a ROS image message.
    :param img: a numpy array
    :return: a ROS image message
    """
    try:
        imgmsg = cv_bridge.CvBridge().cv2_to_imgmsg(img, 'bgr8')
    except cv_bridge.CvBridgeError:
        raise
    return imgmsg
