# -*- coding: utf-8 -*-

""" Module for things related to the control of the baxter research robot.

Implements a single-input-single-output PID controller.

Implements a bang-bang interpolator.

Implements helper classes for handling robot limb poses, configurations and
durations required for the interpolator.
"""

from pid import (
    PidController,
    DIRECT as PID_DIRECT,
    REVERSE as PID_REVERSE
)

from ipl_bb import BangBangInterpolator

from hdl_pose import (
    PoseHandler,
    sample_from_workspace
)
from hdl_config import ConfigurationHandler
from hdl_duration import DurationHandler
