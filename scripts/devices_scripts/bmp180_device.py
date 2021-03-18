#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from std_msgs.msg import String, Float64
from sensor_msgs.msg import Temperature
import serial
from bmp180_driver_py2 import get_bmp180_data
from custom_logger import CustomLogger
import re, sys, traceback
import time
import numpy


class BMP180DeviceServer(object):
    """

    """
    def __init__(self):

        # start node
        rospy.init_node('bmp180_device_server', log_level=rospy.DEBUG)
        # get roslaunch params and reinit part of params
        self._logname = rospy.get_param('bmp180_log_name', 'bmp180')
        self._lost_data_marker = rospy.get_param('bmp180_lost_data_marker', -65536)
        self._log_node_name = rospy.get_param('bmp180_log_node_name', 'bmp180_log_node')
        self._pressure_pub_name = rospy.get_param('bmp180_pressure_pub_name', 'bmp180_1_pressure_pub')
        self._temp_pub_name = rospy.get_param('bmp180_temp_pub_name', 'bmp180_1_temp_pub')

        self._timeout = rospy.get_param('bmp180_timeout', 1)
        self._measure_interval = rospy.get_param('bmp180_measure_interval', 3)

        # create log topic publisher
        self._log_pub = rospy.Publisher(self._log_node_name, String, queue_size=10)

        self._pressure_pub = rospy.Publisher(self._pressure_pub_name, Temperature, queue_size=10)
        self._temp_pub = rospy.Publisher(self._temp_pub_name, Temperature, queue_size=10)

        # logger
        self._logger = CustomLogger(name=self._logname, logpub=self._log_pub)
        self._logger.debug("bmp180_device_server init")
        # print("we here init")

        # and go to infinite loop
        # self._service = rospy.Service(self._service_name, SBA5Device, self.handle_request)
        self._loop()

    def _loop(self):
        rospy.sleep(self._measure_interval)
        while not rospy.is_shutdown():
            try:
                temp, pressure, altitude = get_bmp180_data()

                msg = Temperature()
                msg.temperature = temp
                msg.header.stamp.secs = rospy.get_time()
                self._temp_pub.publish(msg)

                msg = Temperature()
                msg.temperature = pressure
                msg.header.stamp.secs = rospy.get_time()
                self._pressure_pub.publish(msg)

            except Exception as e:
                msg = Temperature()
                msg.temperature = self._lost_data_marker
                msg.header.stamp.secs = rospy.get_time()
                self._temp_pub.publish(msg)

                self._pressure_pub.publish(msg)

            # print("we alive")


if __name__ == "__main__":
    BMP180DeviceServer()

