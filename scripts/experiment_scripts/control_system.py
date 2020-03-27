#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from std_msgs.msg import String
from data_scripts.custom_logger import CustomLogger
from ros_farmer_pc.srv import ControlSystem, LedDeviceResponse, RelayDeviceResponse
import time
from future import *

# custom errors


class ControlSystemException(Exception):
    pass


class ControlSystem(object):
    """

    """
    def __init__(self):
        # hardcoded constants
        self._success_response = "success: "
        self._error_response = "error: "

        # start node
        rospy.init_node('control_server')

        # get roslaunch params

        # names for self topics
        self._logname = rospy.get_param('~control_log_name', 'control_system')
        self._log_node_name = rospy.get_param('~control_log_node_name', 'control_log_node')
        self._service_name = rospy.get_param('~control_service_name', 'control_system')
        self._raw_co2_pub_name = rospy.get_param('~control_raw_co2_pub_name', 'raw_co2_pub')

        # devices services names
        self._relay_service_name = rospy.get_param('~control_relay_service_name', 'relay_device')
        self._led_service_name = rospy.get_param('~control_led_service_name', 'led_device')
        self._co2_service_name = rospy.get_param('~control_co2_service_name', 'sba5_device') # we are using sba5 by default co2 sensor

        # experiment params
        self._full_experiment_loop_time = rospy.get_param('~control_full_experiment_loop_time', 900.0) # sec
        self._isolated_measure_time = rospy.get_param('~control_isolated_measure_time', 480.0) # sec
        self._n2_calibration_time = rospy.get_param('~control_n2_calibration_time', 40.0) # depends on
        self._air_valves_open_time = rospy.get_param('~control_air_valves_open_time', 15) # sec
        # flags of current control regime
        self._mode = rospy.get_param('~control_start_mode', 'experiment')
        # mode can be :
        # experiment
        # life_support
        # ...
        self._at_work = rospy.get_param('~control_start_at_work', True)
        # start node and loop immediately after launch or


        # create log topic publisher
        self._log_pub = rospy.Publisher(self._log_node_name, String, queue_size=10)

        # logger
        self._logger = CustomLogger(name=self._logname, logpub=self._log_pub)
        self._logger.debug("control_server init")

        # service
        self._service = rospy.Service(self._service_name, ControlSystem, self._handle_request)
        self._loop()

    # ===================== loops for different control modes ======================

    def _loop(self):
        # check if mode was changed
        if self._mode == 'experiment':
            self._experiment_loop()
        elif self._mode == 'life_support':
            self._life_support_loop()
        pass


    def _experiment_loop(self):
        # one loop
        # all experiment
        t = time.localtime()
        # every 15 minutes by default
        if t.tm_min % (self._full_experiment_loop_time/60) == 0:
            # start it again
            self._update_control_params()
        pass

    def _update_control_params(self):
        pass

    def _life_support_loop(self):
        # one loop
        pass
        # only air conditioning and light, no other things


    def _handle_request(self):
        pass