#!/usr/bin/env python

import rospy

from std_msgs.msg import Float64

from recorder.queue_recorder import QueueRecorder


def listener():
    rospy.init_node('listener', anonymous=True)

    rec = QueueRecorder(topic='chatter', msg_type=Float64)
    rospy.on_shutdown(rec.clean_shutdown)
    rec.start()
    rospy.sleep(10)
    rec.stop()
    rospy.sleep(1)
    rec.start()
    rospy.sleep(10)
    rec.stop()

    rospy.spin()

if __name__ == '__main__':
    listener()
