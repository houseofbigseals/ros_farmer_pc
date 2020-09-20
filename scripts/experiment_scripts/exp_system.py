#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from std_msgs.msg import String
import traceback
import sys
from custom_logger import CustomLogger
from ros_farmer_pc.srv import ExpSystem


class ExpSystemServer(object):
    """
    heh
    """
    def __init__(self):
        # hardcoded constants
        self._success_response = "success: "
        self._error_response = "error: "

        # start node
        rospy.init_node('exp_server', log_level=rospy.DEBUG)

        # get roslaunch params

        # names for self topics
        self._logname = rospy.get_param('~exp_log_name', 'exp_system')
        self._log_node_name = rospy.get_param('~exp_log_node_name', 'exp_log_node')
        self._service_name = rospy.get_param('~exp_service_name', 'exp_system')
        self._exp_pub_name = rospy.get_param('~exp_pub_name', 'exp_pub')

        # create log topic publisher
        self._log_pub = rospy.Publisher(self._log_node_name, String, queue_size=10)
        # create data topic publisher
        # self._co2_pub = rospy.Publisher(self._raw_co2_pub_name, Temperature, queue_size=10)

        # logger
        self._logger = CustomLogger(name=self._logname, logpub=self._log_pub)
        self._logger.info("exp_server init")

        # service
        self._service = rospy.Service(self._service_name, ControlSystem, self._handle_request)