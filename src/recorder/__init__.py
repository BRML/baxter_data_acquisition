# -*- coding: utf-8 -*-

""" Module for recorders recording data with the baxter research robot.

Implements a camera recorder for recording videos and time stamps for the
image frames from the robot's head camera, as well as a joint recorder for
recording joint configurations, joint torques, Cartesian acceleration as well
as anomaly commands from one of the robot's limbs.
Furthermore, a flash recorder recording time stamps of screen flashes and
two depth recorders (Kinect V2 and Senz3d) recording rgb data analogous to
the camera recorder and depth data as compressed binary files with associated
time stamp files are implemented.
"""

from camera_recorder import (
    CameraClient,
    CameraRecorder
)

from joint_recorder import (
    JointClient,
    JointRecorder
)

from recorder_server import RecorderServer

from flash_recorder import (
    FlashClient,
    FlashRecorder
)

from kinect_recorder import (
    KinectClient,
    KinectRecorder
)
from senz_recorder import (
    SenzClient,
    SenzRecorder
)
