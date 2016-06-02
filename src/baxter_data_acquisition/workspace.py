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

from collections import Sequence
import numpy as np
import rospy

from visualization_msgs.msg import Marker, MarkerArray


class Workspace(Sequence):
    def __init__(self, r1=0.40, r2=0.80, v1=0.40, v2=0.40):
        self._clusters = list()
        self._cluster_workspace(r1, r2, v1, v2)

    def __getitem__(self, item):
        return self._clusters[item]

    def __len__(self):
        return len(self._clusters)

    def _cluster_workspace(self, r1, r2, v1, v2):
        """ Cut the workspace of the baxter robot into several clusters.
        Two cylinders of radius r1 and r2, respectively, are placed around
        the origin of the robot's coordinate frame (oriented along the z-
        axis). The inner cylinder (r1) is intersected at [0, 45, ..., 360)
        degrees, where 0 degrees is along the x-axis. The outer cylinder (r2)
        is intersected at [22.5, 67.5, ..., 360) degrees, where 0 degrees is
        along the x-axis.
        A regular grid is created by combining these points with heights
        [0, +-v1, +-2v1, ...] and [+-v2/2, +-2v2/2, ...], respectively.
        """
        phi = np.arange(start=0, stop=2*np.pi, step=np.pi/4)

        def grid(r):
            return zip(r*np.cos(phi), r*np.sin(phi))

        def mesh(r, v):
            xy = grid(r)
            xyz = list()
            for x, y in xy:
                # TODO set proper upper value
                for z in np.arange(0, 1.43, v):
                    xyz.append((x, y, z))
                # TODO set proper lower value
                for z in np.arange(-v, -0.92, -v):
                    xyz.append((x, y, z))
            return xyz

        self._clusters = np.array(mesh(r1, v1) + mesh(r2, v2/2))

    def cluster_position(self, position):
        """ Classify a given position according to the closest cluster in the
        workspace.
        :param position: A position [x, y, z].
        :returns: The cluster index (==label) for the position.
        """
        dists = np.linalg.norm(self._clusters - np.asarray(position), axis=1)
        return np.argmin(dists, axis=0)

    def visualize_rviz(self):
        pub = rospy.Publisher('workspace_visualization_array', MarkerArray, queue_size=100)
        marker_array = MarkerArray()
        idx = 0
        # for cluster in self._clusters:
        #     marker = Marker()
        #     marker.header.frame_id = "base"
        #     marker.id = idx
        #     marker.type = marker.SPHERE
        #     marker.action = marker.ADD
        #     marker.scale.x = 1.2
        #     marker.scale.y = 1.2
        #     marker.scale.z = 1.2
        #     marker.color.a = .5
        #     marker.color.r = 0.0
        #     marker.color.g = 1.0
        #     marker.color.b = 0.0
        #     marker.pose.orientation.w = 1.0
        #     marker.pose.position.x = cluster[0]
        #     marker.pose.position.y = cluster[1]
        #     marker.pose.position.z = cluster[2]
        marker = Marker()
        marker.header.frame_id = "base"
        marker.id = idx
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 1.2
        marker.scale.y = 1.2
        marker.scale.z = 1.2
        marker.color.a = .5
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = 1.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        marker_array.markers.append(marker)
        idx += 1

        pub.publish(marker_array)
        return True
