#!/usr/bin/env python

import rospy

from std_msgs.msg import Float64

from recorder.queue_recorder import QueueRecorder


def callback(msg):
    rospy.loginfo(msg)


def listener():
    rospy.init_node('listener', anonymous=True)

    for i in range(2):
        if rospy.is_shutdown():
            break
        rec = QueueRecorder(topic='chatter', msg_type=Float64, callback=callback)
        rospy.on_shutdown(rec.clean_shutdown)
        rec.start()
        rospy.sleep(10)
        rec.stop()
        rospy.sleep(1)

if __name__ == '__main__':
    listener()
