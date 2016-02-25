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


class CollisionSampler(object):
    def __init__(self, probability):
        """ Collection of functions for sampling collisions with baxter.
        :param probability: Probability that a collision takes place.
        :return: A collision sampler instance.
        """
        assert 0.0 <= probability <= 1.0, 'Probability not in [0, 1]'
        self._probability = probability

        self._body_parts = ['shoulder', 's1', 'upper_arm', 'e1', 'lower_arm',
                            'w1', 'palm', 'w2']

    def shall_collide(self):
        """
        Return whether a collision is supposed to happen or not. Has a
        self._probability chance to return True.
        ":return Whether a collision is due.
        """

        if rnd.uniform(0.0, 1.0) <= self._probability:
            return True
        return False

    def sample_body_part(self):
        """
        Return the body part on which a collision is supposed to happen.
        :return A string representing the body part.
        """
        draw = rnd.randint(0, len(self._body_parts))
        return self._body_parts[draw]

    def part2int(self, part):
        """ Convert string representation of body part to int representation.
        :param part: String representation of the body part.
        :return: Int representation of the body part.
        """
        try:
            ret = self._body_parts.index(part)
        except ValueError:
            print "ERROR-part2int-Requested part does not exist."
            raise
        return ret
