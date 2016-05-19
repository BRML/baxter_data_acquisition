import rospy

from baxter_data_acquisition.srv import Trigger


class BagClient(object):
    def __init__(self, service_name='bag_service'):
        self._service_name = service_name

    def start(self, outname):
        """ Start bag recorder hosted on bag recorder server.
        :param outname: Filename to write the bag file into.
        :return: (bool success, string message)
        """
        rospy.logdebug("Waiting for bag recorder server.")
        rospy.wait_for_service(self._service_name)
        try:
            trigger = rospy.ServiceProxy(self._service_name, Trigger)
            resp = trigger(on=True, outname=outname)
            return resp.success, resp.message
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s' % e)

    def stop(self):
        """ Stop camera recorder hosted on camera recorder server.
        :return: (bool success, string message)
        """
        rospy.wait_for_service(self._service_name)
        try:
            trigger = rospy.ServiceProxy(self._service_name, Trigger)
            resp = trigger(on=False)
            return resp.success, resp.message
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s' % e)
