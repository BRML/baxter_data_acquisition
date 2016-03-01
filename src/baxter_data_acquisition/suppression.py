import rospy
import threading

from std_msgs.msg import Empty


class Suppressor(threading.Thread):
    def __init__(self, limb, name):
        """ Suppressing collision avoidance or collision detection on one of
        baxter's limbs using a separate thread.
        :param limb: The limb to suppress functionality on.
        :param name: The string identifying the functionality.
        :return: A Suppressor Thread instance.
        """
        super(Suppressor, self).__init__()
        if limb not in ['left', 'right']:
            raise ValueError("'limb' must be 'left' or 'right'!")

        self._stop = False
        self._pub = rospy.Publisher('robot/limb/'+limb+name, Empty)

    def run(self):
        """ Send suppress signal at rate > 5 Hz. """
        while not rospy.is_shutdown() and not self._stop:
            self._pub.publish()
            rospy.sleep(0.1)

    def stop(self):
        """ End the thread's while loop sending the suppress signal.
        :return: Whether the tread is alive.
        """
        self._stop = True
        self.join(5.0)
        return self.is_alive()


class AvoidanceSuppressor(Suppressor):
    def __init__(self, limb):
        """ Suppressing collision avoidance on one of baxter's limbs using a
        separate thread.
        :param limb: The limb to suppress collision avoidance on.
        :return: A Suppressor Thread instance.
        """
        name = '/suppress_collision_avoidance'
        super(AvoidanceSuppressor, self).__init__(limb, name)


class DetectionSuppressor(Suppressor):
    def __init__(self, limb):
        """ Suppressing collision detection on one of baxter's limbs using a
        separate thread.
        :param limb: The limb to suppress collision detection on.
        :return: A Suppressor Thread instance.
        """
        name = '/suppress_contact_safety'
        super(DetectionSuppressor, self).__init__(limb, name)
