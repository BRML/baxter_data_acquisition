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

import numpy.random as rnd
import os
import rospkg
import rospy

from baxter_core_msgs.msg import JointCommand
from std_msgs.msg import (
    Float64MultiArray,
    UInt16
)

import baxter_interface
from baxter_interface import CHECK_VERSION

from baxter_data_acquisition.face import send_image
from baxter_data_acquisition.sampler import CollisionSampler
import baxter_data_acquisition.settings as settings
from baxter_data_acquisition.suppression import (
    AvoidanceSuppressor,
    DetectionSuppressor
)

from recorder import (
    CameraClient,
    JointRecorder
)


class JointPosition(object):
    def __init__(self, limb, number, collisions, images, sim):
        """ Joint position data acquisition with manual induced collisions.
        :param limb: The limb to record data from.
        :param number: The number of samples to record.
        :param collisions: Whether there are collisions in the data.
        :param images: Whether images are to be recorded.
        :param sim: Whether in simulation or reality.
        :return: A baxter robot instance.
        """
        self._arm = limb
        self._number = number
        self._collisions = collisions
        self._images = images
        self._sim = sim

        self._limb = baxter_interface.Limb(self._arm)
        self._rec_joint = JointRecorder(limb=self._arm,
                                        rate=settings.recording_rate,
                                        anomaly_mode='manual')
        self._head = baxter_interface.Head()

        if self._images:
            cam = 'head_camera'
            self._camera = baxter_interface.CameraController(cam, self._sim)
            self._rec_cam = CameraClient()

        self._pub_rate = rospy.Publisher('robot/joint_state_publish_rate',
                                         UInt16, queue_size=10)
        ns = 'data/limb/' + self._arm
        self._pub_cfg_des = rospy.Publisher(ns + '/cfg/des', JointCommand,
                                            queue_size=10)

        self._previous_config = None
        if self._collisions:
            self._sampler = CollisionSampler(settings.probability)
            self._pub_anom = rospy.Publisher(ns + '/anomaly',
                                             Float64MultiArray, queue_size=10)
        self._imgpath = os.path.join(rospkg.RosPack().
                                     get_path('baxter_data_acquisition'),
                                     'share', 'images')

        print "\nGetting robot state ... "
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print "Enabling robot... "
        self._rs.enable()

        send_image(os.path.join(self._imgpath, 'clear.png'))
        self._limb.set_joint_position_speed(0.3)
        self._pub_rate.publish(settings.recording_rate)
        if self._images:
            # Camera handling is one fragile thing...
            try:
                baxter_interface.CameraController('right_hand_camera',
                                                  self._sim).close()
            except AttributeError:
                pass
            self._camera.resolution = (1280, 800)
            self._camera.fps = 14

    def clean_shutdown(self):
        """ Clean shutdown of the robot.
        :return: True on completion
        """
        print "\nExiting joint position collision daq ..."
        send_image(os.path.join(self._imgpath, 'clear.png'))
        self._limb.set_joint_position_speed(0.3)
        self._pub_rate.publish(100)
        self._limb.move_to_neutral()
        if not self._init_state:
            print "Disabling robot..."
            self._rs.disable()
        return True

    def execute(self, outfile):
        """ Recording of collision data with the baxter research robot.
        :param outfile: path and filename of the file(s) to write the data to,
        without the extension(s).
        """
        threads = [
            AvoidanceSuppressor(self._arm),
            DetectionSuppressor(self._arm)
        ]
        for thread in threads:
            thread.start()

        print '\nRecord data %s collisions into %s.' % \
              ('with' if self._collisions else 'without', outfile)
        self._head.set_pan(0.0)
        self._limb.move_to_neutral()
        try:
            for nr in range(self._number):
                if rospy.is_shutdown():
                    break
                print 'Recording sample %i of %d.' % (nr + 1, self._number)

                self._rec_joint.start(outfile)
                if self._images:
                    self._rec_cam.start(outfile + '-%i' % nr,
                                        self._camera.fps,
                                        self._camera.resolution)
                self._one_sample()
                if self._images:
                    self._rec_cam.stop()
                self._rec_joint.stop()
                self._rec_joint.write_sample()
        except rospy.ROSInterruptException:
            pass
        for thread in threads:
            ret = thread.stop()
        rospy.signal_shutdown('Done with experiment.')

    def _one_sample(self):
        """ One sample with manually induced collisions.

        Baxter moves one limb randomly between pre-defined configurations for
        a given period of time. For each movement it is randomly selected
        whether a collision (=anomaly) is due. If yes, a body part of the limb
        is selected at random, visualized on the head display, and the
        operator instructed to hit the pointed out body part as soon as
        possible by baxter nodding its head.
        :return: True on completion.
        """
        elapsed = 0.0
        start = rospy.get_time()
        while not rospy.is_shutdown() and elapsed < settings.run_time:
            cmd = self._sample_configuration()
            if self._collisions:
                if self._sampler.shall_collide():
                    part = self._sampler.sample_body_part()
                    print '\tInduce collision on %s arm at %s' % \
                          (self._arm, part)
                    self._head.command_nod(timeout=0.0)
                    send_image(os.path.join(self._imgpath,
                                            'hit_%s.png' % part))
                    self._pub_anom.publish(data=[self._sampler.part2int(part),
                                                 ])
                else:
                    send_image(os.path.join(self._imgpath, 'clear.png'))
            self._pub_cfg_des.publish(
                command=[cmd[j] for j in self._rec_joint.get_header_cfg()[1:]])
            self._limb.move_to_joint_positions(cmd)
            elapsed = rospy.get_time() - start

        send_image(os.path.join(self._imgpath, 'clear.png'))
        self._limb.move_to_neutral()
        return True

    def _sample_configuration(self):
        """ Randomly selects one of the configurations stored in
        self._configurations.
        """
        config = None
        while True:
            config = rnd.randint(0, 9)
            if not config == self._previous_config:
                break
        self._previous_config = config
        return self._configurations(config)

    def _configurations(self, configuration):
        """ Returns the desired configuration from the list of configurations.
        :type configuration: int
        :param configuration: The id of the configuration [0, 9].
        """
        configs = [{self._arm + '_w0': -0.15071361223754884,
                    self._arm + '_w1': 1.1017816996398926,
                    self._arm + '_w2': 0.059058260266113285,
                    self._arm + '_e0': -1.5562235075317383,
                    self._arm + '_e1': 1.948155598388672,
                    self._arm + '_s0': -0.42798063933105474,
                    self._arm + '_s1': -0.07363107773437501},
                   {self._arm + '_w0': -0.2949078061340332,
                    self._arm + '_w1': 0.9633399336914064,
                    self._arm + '_w2': 0.388864129284668,
                    self._arm + '_e0': -3.0349809853637697,
                    self._arm + '_e1': 1.1198059738769532,
                    self._arm + '_s0': 0.7804127249450684,
                    self._arm + '_s1': -0.9127185677490235},
                   {self._arm + '_w0': 2.319762443829346,
                    self._arm + '_w1': -0.13575729957275393,
                    self._arm + '_w2': 3.052621764404297,
                    self._arm + '_e0': -0.1817767231567383,
                    self._arm + '_e1': 0.779645734552002,
                    self._arm + '_s0': -0.3179175179260254,
                    self._arm + '_s1': 0.4613447214294434},
                   {self._arm + '_w0': 0.16030099215087892,
                    self._arm + '_w1': 1.3261263896118165,
                    self._arm + '_w2': 1.0941117957092286,
                    self._arm + '_e0': 0.4970097747070313,
                    self._arm + '_e1': 1.4645681555603027,
                    self._arm + '_s0': 1.234471037640381,
                    self._arm + '_s1': 0.15186409782714844},
                   {self._arm + '_w0': -1.6037769119018557,
                    self._arm + '_w1': -0.2688301327697754,
                    self._arm + '_w2': 2.1303158167419434,
                    self._arm + '_e0': 0.44485442797851565,
                    self._arm + '_e1': 1.6958157590698244,
                    self._arm + '_s0': 0.898529245477295,
                    self._arm + '_s1': -0.5794612419616699},
                   {self._arm + '_w0': 0.22396119477539064,
                    self._arm + '_w1': 0.6879903825805664,
                    self._arm + '_w2': 3.0530052596008304,
                    self._arm + '_e0': -1.1224904402526856,
                    self._arm + '_e1': 0.8713010865234375,
                    self._arm + '_s0': -1.1393642289001467,
                    self._arm + '_s1': -0.3459126672729492},
                   {self._arm + '_w0': -0.04333495720825196,
                    self._arm + '_w1': 0.39193209085693365,
                    self._arm + '_w2': 3.0537722499938966,
                    self._arm + '_e0': -0.42222821138305666,
                    self._arm + '_e1': 0.9763787703735353,
                    self._arm + '_s0': -0.9840486743041993,
                    self._arm + '_s1': 0.4103398602905274},
                   {self._arm + '_w0': -0.45405831269531255,
                    self._arm + '_w1': 1.5600584594970703,
                    self._arm + '_w2': 2.744675121588135,
                    self._arm + '_e0': -2.059752700579834,
                    self._arm + '_e1': 1.7192089660583498,
                    self._arm + '_s0': 0.04640291878051758,
                    self._arm + '_s1': -0.3087136332092285},
                   {self._arm + '_w0': -0.562587453314209,
                    self._arm + '_w1': 2.0942672682678225,
                    self._arm + '_w2': 1.9573594831054688,
                    self._arm + '_e0': -1.8844953957641604,
                    self._arm + '_e1': 1.7353157643127441,
                    self._arm + '_s0': 0.10469418865356446,
                    self._arm + '_s1': 0.11773302533569337},
                   {self._arm + '_w0': 0.2439029449951172,
                    self._arm + '_w1': 1.459582718005371,
                    self._arm + '_w2': 3.0537722499938966,
                    self._arm + '_e0': -0.10277671267089844,
                    self._arm + '_e1': 1.5608254498901368,
                    self._arm + '_s0': 1.5002332088378907,
                    self._arm + '_s1': -0.04256796681518555}]
        return configs[configuration]
