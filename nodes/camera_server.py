#!/usr/bin/env python

import rospy

from baxter_data_acquisition.srv import Trigger, TriggerResponse
from recorder import CameraRecorder


class Handler(object):
    def __init__(self):
        self._cr = CameraRecorder()
        self._running = False

    def handle_trigger(self, req):
        if req.on and not self._running:
            self._running = True
            resp = self._cr.start(outname=req.outname, fps=req.fps, imgsize=req.size)
            msg = "Started camera recorder"
        elif not req.on and self._running:
            self._running = False
            resp = self._cr.stop()
            msg = "Stopped camera recorder"
        else:
            resp = False
            msg = "Already/not yet running"
        print msg
        return TriggerResponse(success=resp, message=msg)


if __name__ == "__main__":
    rospy.init_node('camera_server')
    h = Handler()
    s = rospy.Service('camera_server', Trigger, h.handle_trigger)
    print 'Ready to get triggered.'
    rospy.spin()
