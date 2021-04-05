#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from sensor_msgs.msg import Temperature
from ds18b20_driver_py2 import ds18b20_read_temp
from custom_logger import CustomLogger


class DS18B20DeviceServer(object):
    """

    """
    def __init__(self):

        # start node
        rospy.init_node('ds18b20_device_server', log_level=rospy.DEBUG, anonymous=True)
        # get roslaunch params and reinit part of params
        self._device_name = rospy.get_param('~ds18b20_device_name')
        self._logname = 'ds18b20_' + self._device_name[3:] #rospy.get_param('~ds18b20_log_name', 'ds18b20')
        self._lost_data_marker = rospy.get_param('~ds18b20_lost_data_marker', -65536)
        # self._log_node_name = rospy.get_param('~ds18b20_log_node_name', 'ds18b20_log_node')
        # self._temp_pub_name = rospy.get_param('~ds18b20_temp_pub_name', 'ds18b20_1_temp_pub')
        self._temp_pub_name = 'ds18b20_' + self._device_name[3:] + '_data_pub'

        self._timeout = rospy.get_param('~ds18b20_timeout', 1)
        self._measure_interval = rospy.get_param('~ds18b20_measure_interval')

        self._temp_pub = rospy.Publisher(self._temp_pub_name, Temperature, queue_size=10)

        # logger
        self._logger = CustomLogger(name=self._logname)
        self._logger.info("ds18b20_device_server init")
        # print("we here init")

        # and go to infinite loop
        self._loop()

    def _loop(self):
        while not rospy.is_shutdown():
            rospy.sleep(self._measure_interval)
            try:
                temp = ds18b20_read_temp(self._device_name)

                msg = Temperature()
                msg.temperature = temp
                msg.header.stamp.secs = rospy.get_time()
                self._temp_pub.publish(msg)

            except Exception as e:
                msg = Temperature()
                msg.temperature = self._lost_data_marker
                msg.header.stamp.secs = rospy.get_time()
                self._temp_pub.publish(msg)

            # print("we alive")


if __name__ == "__main__":
    DS18B20DeviceServer()

