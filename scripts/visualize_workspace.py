#!/usr/bin/env python

import argparse
import rospy

from baxter_data_acquisition.workspace import Workspace


def main():
    """ Visualize the workspace clusters in RVIZ.

    Usage:
    # Load Baxter model
    $ roslaunch baxter_data_acquisition simulation.launch
    # Start RVIZ
    $ rosrun rviz rviz
    # Visualize Workspace
    $ rosrun baxter_data_acquisition visualize_workspace.py
    # In RVIZ, do `add`, `by topic`, `/workspace_marker_array` and `add`
    # `RobotModel`
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    args = parser.parse_args(rospy.myargv()[1:])

    print 'Initializing node ...'
    rospy.init_node('workspace_visualization', anonymous=True)

    ws = Workspace(r1=0.5, r2=0.9, min_z=-0.3, max_z=1.1)
    print "Visualizing ..."
    ws.visualize_rviz()
    print "Done!"


if __name__ == '__main__':
    main()
