#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from std_msgs.msg import String, Float64
from sensor_msgs.msg import Temperature
import serial
from si7021_driver_py2 import get_si7021_data
from custom_logger import CustomLogger
import re, sys, traceback
import time
import numpy


class SI7021DeviceServer(object):
    """

    """
    def __init__(self):

        # start node
        rospy.init_node('si7021_device_server', log_level=rospy.DEBUG)
        # get roslaunch params and reinit part of params
        self._logname = rospy.get_param('~si7021_log_name', 'si7021')
        self._lost_data_marker = rospy.get_param('~si7021_lost_data_marker', -65536)
        self._log_node_name = rospy.get_param('~si7021_log_node_name', 'si7021_log_node')
        self._hum_pub_name = rospy.get_param('~si7021_hum_pub_name', 'si7021_1_hum_pub')
        self._temp_pub_name = rospy.get_param('~si7021_temp_pub_name', 'si7021_1_temp_pub')

        self._timeout = rospy.get_param('~si7021_timeout', 1)
        self._measure_interval = rospy.get_param('~si7021_measure_interval')

        # create log topic publisher
        self._log_pub = rospy.Publisher(self._log_node_name, String, queue_size=10)

        self._hum_pub = rospy.Publisher(self._hum_pub_name, Temperature, queue_size=10)
        self._temp_pub = rospy.Publisher(self._temp_pub_name, Temperature, queue_size=10)

        # logger
        self._logger = CustomLogger(name=self._logname, logpub=self._log_pub)
        self._logger.debug("si7021_device_server init")
        # print("we here init")

        # and go to infinite loop
        # self._service = rospy.Service(self._service_name, SBA5Device, self.handle_request)
        self._loop()

    def _loop(self):
        while not rospy.is_shutdown():
            rospy.sleep(self._measure_interval)
            try:
                temp, hum = get_si7021_data()

                msg = Temperature()
                msg.temperature = temp
                msg.header.stamp.secs = rospy.get_time()
                self._temp_pub.publish(msg)

                msg = Temperature()
                msg.temperature = hum
                msg.header.stamp.secs = rospy.get_time()
                self._hum_pub.publish(msg)
            except Exception as e:
                msg = Temperature()
                msg.temperature = self._lost_data_marker
                msg.header.stamp.secs = rospy.get_time()
                self._temp_pub.publish(msg)

                self._hum_pub.publish(msg)

            # print("we alive")


if __name__ == "__main__":
    SI7021DeviceServer()

