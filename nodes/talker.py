#!/usr/bin/env python

import rospy

from std_msgs.msg import Float64


def talker():
    pub = rospy.Publisher('chatter', Float64, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        time = rospy.get_time()
        rospy.loginfo('Hello at %f s.' % time)
        pub.publish(time)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
