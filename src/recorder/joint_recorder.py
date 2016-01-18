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

import h5py


class JointRecorder(object):
    def __init__(self, limb, anomaly_mode='manual'):
        """ Joint recorder class writing baxter joint data into a .hdf5 file,
        where the index gives the sample number, containing modalities
        ('configuration', 'effort', 'anomaly', 'acceleration') and their
        corresponding fields ('measured', 'commanded'), if appropriate.
        :param limb: The limb to record data from ['left', 'right', 'both']
        :param anomaly_mode: Type of anomaly in the data ['manual',
        'automatic']
        :return: Joint recorder instance.
        """
        self._header = dict()
        if limb == 'both':
            self._header['configuration'] = \
                "time, left_s0, left_s1, left_e0, left_e1, left_w0, " \
                "left_w1, left_w2, right_s0, right_s1, right_e0, " \
                "right_e1, right_w0, right_w1, right_w2"
            self._header['acceleration'] = \
                "time, left_x, left_y, left_z, right_x, right_y, right_z"
        else:
            self._header['configuration'] = \
                "time, {0}_s0, {0}_s1, {0}_e0, {0}_e1, {0}_w0, {0}_w1, " \
                "{0}_w2".format(limb)
            self._header['acceleration'] = \
                "time, {0}_x, {0}_y, {0}_z".format(limb)
        self._header['effort'] = self._header['configuration']
        if anomaly_mode is 'manual':
            self._header['anomaly'] = \
                "time, partId (0-shoulder, 1-s1, 2-upper arm, 3-e1, " \
                "4-lower arm, 5-w1, 6-palm, 7-w2)"
        else:
            self._header['anomaly'] = ""

        self._filename = None
        self._data = None

    def start(self, outfile):
        """ Start joint data recording.
        :param outfile: Filename to write the data to, without the extension.
        """
        self._filename = outfile + '.h5'
        self._data = dict()

    def stop(self):
        """ Stop joint data recording. """
        pass

    def write_sample(self):
        """ Append data of one sample to the .hdf5 joint data file. """
        with h5py.File(self._filename, 'a') as fp:
            idx = len(fp)
            g = fp.require_group('%i' % idx)
            for modality in self._data:
                gm = g.require_group(modality)
                for field in self._data[modality]:
                    gf = gm.require_dataset(field,
                        shape=self._data[modality][field].shape,
                        dtype=self._data[modality][field].dtype,
                        data=self._data[modality][field])
                    gf.attrs.create('Header', data=self._header[modality])
        print 'Done writing sample data to file.'

    def _cb_state(self, data):
        pass
