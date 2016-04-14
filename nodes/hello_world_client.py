#!/usr/bin/env python

import rospy
import sys

from baxter_data_acquisition.srv import Trigger


def trigger_client(b):
    rospy.wait_for_service('trigger')
    try:
        trigger = rospy.ServiceProxy('trigger', Trigger)
        resp = trigger(on=b)
        return resp.success, resp.message
    except rospy.ServiceException as e:
        print 'Service call failed: %s' % e


def usage():
    return "%s [bool]" % sys.argv[0]


if __name__ == "__main__":
    if len(sys.argv) == 2:
        b = sys.argv[1].lower() in ('start', 'true', '1')
        print b, type(b)
    else:
        print usage()
        sys.exit(1)
    print "Requesting %s." % ('start' if b else 'stop')
    print trigger_client(b)
