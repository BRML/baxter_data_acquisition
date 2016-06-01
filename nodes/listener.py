#!/usr/bin/env python

import rospy

from std_msgs.msg import Float64

from recorder.queue_recorder import QueueRecorder


class Logger(QueueRecorder):
    def __init__(self, topic, msg_type):
        super(Logger, self).__init__(topic=topic, msg_type=msg_type)

    def on_start(self):
        pass

    def on_stop(self):
        pass

    def process_msg(self, msg):
        rospy.loginfo(msg)


def listener():
    rospy.init_node('listener', anonymous=True)

    for i in range(2):
        rec = Logger(topic='chatter', msg_type=Float64)
        rospy.on_shutdown(rec.clean_shutdown)
        rec.start()
        rospy.sleep(10)
        rec.stop()
        rospy.sleep(1)

    rospy.spin()

if __name__ == '__main__':
    listener()
