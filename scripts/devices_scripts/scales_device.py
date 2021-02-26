#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from std_msgs.msg import String, Float64
import serial
from custom_logger import CustomLogger
# from ros_farmer_pc.srv import SBA5Device
import re, sys, traceback
import time
import numpy


class ScalesDeviceServer(object):
    """

    """
    def __init__(self, dev, baud, timeout):

        # start node
        rospy.init_node('scales_device_server', log_level=rospy.DEBUG)
        # get roslaunch params and reinit part of params
        self._logname = rospy.get_param('~scales_log_name', 'scales')
        self._log_node_name = rospy.get_param('~scales_log_node_name', 'scales_log_node')
        self._data_pub_name = rospy.get_param('~scales_data_pub_name', 'scales_data_pub')
        self._port = rospy.get_param('~scales_port',
                                     '/dev/serial/by-id/usb-USB_Vir_USB_Virtual_COM-if00')
        self._baudrate = rospy.get_param('~scales_baudrate', 9600)
        self._timeout = rospy.get_param('~scales_timeout', 1)
        self._measure_interval = rospy.get_param('~scales_measure_interval', 5)

        # create log topic publisher
        self._log_pub = rospy.Publisher(self._log_node_name, String, queue_size=10)

        self._data_pub = rospy.Publisher(self._data_pub_name, Float64, queue_size=10)

        # logger
        self._logger = CustomLogger(name=self._logname, logpub=self._log_pub)
        self._logger.debug("scales_device_server init")
        print("we here init")

        # and go to infinite loop
        # self._service = rospy.Service(self._service_name, SBA5Device, self.handle_request)
        self._loop()

    def _loop(self):
        # rospy.sleep(1)
        while not rospy.is_shutdown():
            res = self.get_mean_data(self._measure_interval)
            self._data_pub.publish(Float64(data=res))
            print("we alive")

    def get_mean_data(self, dtime):
        w_datas = []
        t_start = time.time()
        with serial.Serial(self._port, self._baudrate, timeout=self._timeout) as scales:
            while time.time() - t_start < dtime:
                try:
                    raw_data = scales.readline()
                    pattern = re.compile(r'\w\w,\w\w,\s*(\d+.\d+)\s*\w')  # for answers like "ST, GS, 55.210 g"
                    w_data = float(pattern.findall(raw_data)[0])
                    w_datas.append(w_data)
                except Exception as e:
                    exc_info = sys.exc_info()
                    err_list = traceback.format_exception(*exc_info)
                    self._logger.error("scales serial error: {}".format(err_list))
        res = numpy.mean(w_datas)
        return res

    def get_data(self):
        with serial.Serial(self._port, self._baudrate, timeout=self._timeout) as scales:
            try:
                raw_data = scales.readline()
                pattern = re.compile(r'\w\w, \w\w, (\d+.\d+) \w')  # for answers like "ST, GS, 55.210 g"
                w_data = float(pattern.findall(raw_data)[0])
                return w_data
            except Exception as e:
                exc_info = sys.exc_info()
                err_list = traceback.format_exception(*exc_info)
                self._logger.error("scales serial error: {}".format(err_list))
                return -66536.65


if __name__ == "__main__":
    ScalesDeviceServer()

