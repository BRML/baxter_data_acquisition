#!/usr/bin/env python

import rospy

from recorder.flash_recorder import FlashRecorder


def callback(msg):
    rospy.loginfo(msg)


def listener():
    rospy.init_node('listener', anonymous=True)
    rec = FlashRecorder()
    rospy.on_shutdown(rec.clean_shutdown)

    name = ['/home/mludersdorfer/software/test1.txt', '/home/mludersdorfer/software/test2.txt']
    for i in range(2):
        if rospy.is_shutdown():
            break
        rec.start(outname=name[i])
        rospy.sleep(10)
        rec.stop()
        rospy.sleep(1)

if __name__ == '__main__':
    listener()
