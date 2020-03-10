#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String

from ros_farmer_pc.srv import LedDevice, LedDeviceResponse
from low_level_drivers.led_uart_driver_py2 import UartWrapper


class LedDeviceServer(object):
    """
    class to receive messages from other ros nodes, handle it
    send messages to led driver and parse its output back as response
    """
    def __init__(self):
        # hardcoded constants
        pass

        # start node
        rospy.init_node('led_device_server')
        rospy.loginfo('LedDeviceServer: start node')

        # get roslaunch params and reinit part of params
        self._port = rospy.get_param('~_led_port', '/dev/ttyUSB0')
        self._baudrate = rospy.get_param('~_led_baudrate', 19200)
        self._timeout = rospy.get_param('~_led_timeout', 10)
        self._service_name = rospy.get_param('~_led_service_name', 'led_device')


        # init led uart driver

        self.uart_driver = UartWrapper(self._port, self._baudrate, self._timeout)

    def handle_request(self, req):
        # check params from request
        # at first find what user wants from us:
        if not req.command:
            # if we got empty string
            return 
        pass

    #def led_device_server(self, ss):
    #    rospy.init_node('')


