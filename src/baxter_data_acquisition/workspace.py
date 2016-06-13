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
    def __init__(self, r1=0.50, r2=0.90, min_z=-0.3, max_z=1.1):
        self._clusters = list()
        self._cluster_workspace(r1, r2, min_z=min_z, max_z=max_z)

    def __getitem__(self, item):
        return self._clusters[item]

    def __len__(self):
        return len(self._clusters)

    def _cluster_workspace(self, r1, r2, min_z, max_z):
        """ Cut the workspace of the baxter robot into several clusters.
        Two cylinders of radius r1 and r2, respectively, are placed around
        the origin of the robot's coordinate frame (oriented along the z-
        axis). The inner cylinder (r1) is intersected at [0, 45, ..., 360)
        degrees, where 0 degrees is along the x-axis. The outer cylinder (r2)
        is intersected at [22.5, 67.5, ..., 360) degrees, where 0 degrees is
        along the x-axis.
        A regular grid is created by combining these points with appropriate
        heights.
        """
        def approximate_circle(radius, phi):
            return zip(radius*np.cos(phi), radius*np.sin(phi))

        xyz = list()
        # one cluster above head at maximal height
        xyz.append((0.0, 0.0, max_z))
        # one inner cylinder of clusters
        phi1 = np.arange(start=0, stop=2*np.pi, step=np.pi/3)
        xy = approximate_circle(radius=r1, phi=phi1)
        d = float(max_z - min_z)/5.0
        for z in np.arange(min_z+d, max_z-2*d+0.01, 2*d):
            for x, y in xy:
                xyz.append((x, y, z))
        # one outer cylinder of clusters
        phi2 = np.arange(start=np.deg2rad(22.5), stop=2*np.pi, step=np.pi/4)
        xy = approximate_circle(radius=r2, phi=phi2)
        for z in np.arange(min_z, max_z-d+0.01, 2*d):
            for x, y in xy:
                xyz.append((x, y, z))

        self._clusters = np.array(xyz)

    def cluster_position(self, position):
        """ Classify a given position according to the closest cluster in the
        workspace.
        :param position: A position [x, y, z].
        :returns: The cluster index (==label) for the position.
        """
        dists = np.linalg.norm(self._clusters - np.asarray(position), axis=1)
        return np.argmin(dists, axis=0)

    def visualize_rviz(self):
        """ Visualize the workspace cluster points as a RVIZ MarkerArray. """
        topic = 'workspace_marker_array'
        pub = rospy.Publisher(topic, MarkerArray, queue_size=100)
        marker_array = MarkerArray()

        while not rospy.is_shutdown() and pub.get_num_connections() < 1:
            rospy.sleep(0.5)
        for idx, cluster in enumerate(self._clusters):
            marker = Marker()
            marker.header.frame_id = "world"
            marker.id = idx
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = cluster[0]
            marker.pose.position.y = cluster[1]
            marker.pose.position.z = cluster[2]
            marker_array.markers.append(marker)
        pub.publish(marker_array)
