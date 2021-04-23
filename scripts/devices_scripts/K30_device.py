#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from k30_driver_py2 import K30
from std_msgs.msg import String
from custom_logger import CustomLogger
from ros_farmer_pc.srv import K30Device


class K30DeviceServer:
    """

    """
    def __init__(self):
        # hardcoded constants
        self._success_response = "success: "
        self._error_response = "error: "

        # start node
        rospy.init_node('k30_device_server', log_level=rospy.DEBUG)
        self._logname = rospy.get_param('~k30_log_name', 'k30')
        self._log_node_name = rospy.get_param('~k30_log_node_name', 'k30_log_node')
        self._port = rospy.get_param('~k30_port',
                                     '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_'
                                     'UART_Bridge_Controller_0003-if00-port0')
        self._baudrate = rospy.get_param('~k30_baudrate', 9600)
        self._timeout = rospy.get_param('~k30_timeout', 1)
        self._service_name = rospy.get_param('~k30_service_name', 'k30_device')
        self._calibration_time = rospy.get_param('~k30_calibration_time', 25)
        self._lost_data_marker = rospy.get_param('~k30_lost_data_marker', -65536)

        # create log topic publisher
        self._log_pub = rospy.Publisher(self._log_node_name, String, queue_size=10)

        # logger
        self._logger = CustomLogger(name=self._logname, logpub=self._log_pub)
        self._logger.debug("k30_device_server init")

        # device
        self._device = K30(
            devname=self._port,
            baudrate=self._baudrate,
            timeout=self._timeout
        )

        # service
        self._service = rospy.Service(self._service_name, K30Device, self.handle_request)
        self._loop()

    def _loop(self):
        rospy.spin()

    def handle_request(self, req):
        self._logger.debug("we got request: {}".format(req))

        if not req.command:
            # if we got empty string
            resp = self._error_response + 'empty_command'
            return resp

        elif req.command == 'get_info':
            try:
                ans = self._device.get_info()
                resp = self._success_response + ans
                return resp
            except Exception as e:
                resp = self._error_response + e.args[0]
                return resp

        elif req.command == 'get_data':
            try:
                co2, ans = self._device.get_co2()
                # resp = self._success_response + ans
                return str(co2)
            except Exception as e:
                # resp = self._error_response + e.args[0]
                co2 = self._lost_data_marker
                return str(co2)

        elif req.command == 'recalibrate':
            try:
                ans = self._device.recalibrate(ctime=5)
                resp = self._success_response + ans
                return resp
            except Exception as e:
                resp = self._error_response + e.args[0]
                return resp


if __name__ == "__main__":
    K30DeviceServer()
