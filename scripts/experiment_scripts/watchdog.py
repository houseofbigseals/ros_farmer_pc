#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from ros_farmer_pc.srv import Watchdog
from data_scripts.custom_logger import CustomLogger
from std_msgs.msg import String, Header


class WatchdogServer(object):

    def __init__(self):
        # hardcoded constants
        self._success_response = "success: "
        self._error_response = "error: "

        # start node
        rospy.init_node('watchdog_server',anonymous=True, log_level=rospy.DEBUG)

        # get roslaunch params

        # names for self topics
        self._logname = rospy.get_param('~watchdog_log_name', 'data_saver')
        self._log_node_name = rospy.get_param('~watchdog_log_node_name', 'watchdog')
        self._service_name = rospy.get_param('~watchdog_service_name', 'watchdog')

        # create log topic publisher
        self._log_pub = rospy.Publisher(self._log_node_name, String, queue_size=10)

        # start logger
        self._logger = CustomLogger(name=self._logname, logpub=self._log_pub)
        self._logger.debug("data_saver server init")

        self.dead_flag = False

        # service
        self._service = rospy.Service(self._service_name, Watchdog, self._handle_request)

        self._logger.debug("end of init, go to loop")

        self._loop()

    def _loop(self):
        try:
            while not rospy.core.is_shutdown() and not self.dead_flag:
                rospy.sleep(1)
        except KeyboardInterrupt:
            self._logger.warning("keyboard interrupt, shutting down")
            rospy.core.signal_shutdown('keyboard interrupt')


    def _handle_request(self, req):
        self._logger.debug("+++++++++++++++++++++++++++++++++++++++++++++++")
        self._logger.debug("+++++++++++++++++++++++++++++++++++++++++++++++")
        self._logger.debug("we got request: {}".format(req))

        if not req.command:
            # if we got empty string
            resp = self._error_response + 'empty_command'
            return resp

        elif req.command == 'shutdown':
            self.dead_flag = True
            resp = self._success_response + "self.dead_flag is {}".format(self.dead_flag)
            return resp

        # elif req.command == 'free_lock':
        #     self._data_write_lock.release()
        #     resp = self._success_response + "the lock has been released"
        #     # lock will be acqured forever if we dont call free_lock command
        #     return resp

        else:
            resp = self._error_response + 'unknown command'
            return resp


if __name__ == "__main__":
    WatchdogServer()