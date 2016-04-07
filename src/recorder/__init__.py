# -*- coding: utf-8 -*-

""" Module for recorders recording data with the baxter research robot.

Implements a camera recorder for recording videos and time-stamps for the
image frames from the robot's head camera, as well as a joint recorder for
recording joint configurations, joint torques, Cartesian acceleration as well
as anomaly commands from one of the robot's limbs.
"""

from camera_recorder import CameraRecorder

from joint_recorder import JointRecorder

from flash_recorder import FlashRecorder

from kinect_recorder import KinectRecorder
from senz_recorder import SenzRecorder
