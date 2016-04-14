#!/usr/bin/env python

import rospy

from baxter_data_acquisition.srv import Trigger, TriggerResponse


def start():
    print 'Start'
    return TriggerResponse(success=True, message="I did start!")


def stop():
    print 'Stop'
    return TriggerResponse(success=True, message="I did stop!")


def handle_trigger(req):
    if req.on:
        resp = start()
    else:
        resp = stop()
    return resp
    #
    # print 'Hello World!'
    # rospy.sleep(10.0)
    # return TriggerResponse(success=True, message="I did say 'Hello World!'")


def trigger_server():
    rospy.init_node('trigger_server')
    s = rospy.Service('trigger', Trigger, handle_trigger)
    print 'Ready to get triggered.'
    rospy.spin()


if __name__ == "__main__":
    trigger_server()
