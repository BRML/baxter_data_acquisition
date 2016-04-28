# -*- coding: utf-8 -*-

""" Module in which different experiments with the baxter research robot are
    implemented.


collision.py implements manually induced collisions on position-controlled
    motions of one limb of the robot.

anomaly.py implements automatically induced anomalies by means of control
    parameter modifications of the position controller of one limb of the
    robot at runtime.

goal.py implements torque-controlled pseudo-random motions of one limb of the
    robot.

handshake.py implements velocity-controlled wobble-and-occlude-my-hands-motion
    or a position-controlled human-robot handshake simulation, depending on
    which experiment is supposed to run.
"""
