#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from std_msgs.msg import String
from data_scripts.custom_logger import CustomLogger
from ros_farmer_pc.srv import ControlSystem, LedDevice, RelayDevice, SBA5Device
from sensor_msgs.msg import Temperature
import time
import re
# from future import *

# custom errors


class ControlSystemException(Exception):
    pass


class ControlSystemServer(object):
    """

    here we are using rospy.Timer functionality to measure co2 every second
    https://roboticsbackend.com/how-to-use-a-ros-timer-in-python-to-publish-data-at-a-fixed-rate/
    https://roboticsbackend.com/ros-rate-roscpy-roscpp/
    but every global reconfiguration cycle works with time.localtime()
    why
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
        self._sba5_service_name = rospy.get_param('~control_sba5_service_name', 'sba5_device')  # we are using sba5 by default co2 sensor

        # experiment params
        self._default_red = rospy.get_param('~control_default_red', 50)  # mA
        self._default_white = rospy.get_param('~control_default_white', 50)  # mA
        self._full_experiment_loop_time = rospy.get_param('~control_full_experiment_loop_time', 900.0) # sec
        self._isolated_measure_time = rospy.get_param('~control_isolated_measure_time', 480.0)  # sec
        self._n2_calibration_time = rospy.get_param('~control_n2_calibration_time', 40.0)  # depends on
        self._air_valves_open_time = rospy.get_param('~control_air_valves_open_time', 15.0)  # sec
        self._co2_measure_time = rospy.get_param('~control_co2_measure_time', 1.0)  # sec
        self._ventilation_time = self._full_experiment_loop_time - self._isolated_measure_time - \
            self._n2_calibration_time - self._air_valves_open_time

        # flags of current control regime
        self._mode = rospy.get_param('~control_start_mode', 'life_support')
        # mode can be :
        # experiment
        # life_support
        # ...
        self._sba5_measure_allowed = False  # by default
        self._at_work = rospy.get_param('~control_start_at_work', True)
        # start node and loop immediately after launch or


        # create log topic publisher
        self._log_pub = rospy.Publisher(self._log_node_name, String, queue_size=10)
        # create data topic publisher
        self._co2_pub = rospy.Publisher(self._raw_co2_pub_name, Temperature, queue_size=10)

        # logger
        self._logger = CustomLogger(name=self._logname, logpub=self._log_pub)
        self._logger.debug("control_server init")

        # create timers for async periodic tasks using internal ros mechanics
        # Create a ROS Timer for reading data
        rospy.Timer(rospy.Duration(1.0), self._get_sba5_measure())  # 1 Hz
        # create ros timer for main loop

        # TODO connect to led and relay services
        # or not

        # service
        self._service = rospy.Service(self._service_name, ControlSystem, self._handle_request)
        self._loop()

    # ===================== loops for different control modes ======================

    def _loop(self):
        while not rospy.is_shutdown():
            # check if mode was changed
            if self._mode == 'experiment':
                self._experiment_loop()
            elif self._mode == 'life_support':
                self._life_support_loop()
            pass

    def _life_support_loop(self):
        # one loop

        t = time.localtime()
        # every 15 minutes by default
        if t.tm_min % (self._full_experiment_loop_time / 60.0) == 0:
            self._logger.debug("start life support loop again")
            # start it again
            # reset light parameters to default
            self._set_new_light_mode(self._default_red, self._default_white)
            # set inside ventilation coolers on
            self._set_new_relay_state('set_vent_coolers', 0)
            # start ventilation
            self._start_ventilation()
            # wait for self._ventilation_time
            rospy.sleep(self._ventilation_time)  # its not bad because service calls works in parallel threads
            # then stop it
            self._stop_ventilation()
            # then wait and do nothing


    def _experiment_loop(self):
        # one loop
        # all experiment
        t = time.localtime()
        # every 15 minutes by default
        if t.tm_min % (self._full_experiment_loop_time/60.0) == 0:
            # start it again
            # get new regime
            self._update_control_params()
            # start ventilation and calibration
            self._start_ventilation()
            # stop measuring co2
            # do calibration of sba-5
            self._perform_sba5_calibration()
            # start measuring co2 again
            # wait for self._ventilation_time
            vent_done_flag = False
            vent_start_time = time.localtime()
            while not vent_done_flag:
                pass

            # stop
        pass

    # =============================== support methods ==============================


    def _update_control_params(self):
        # get new params and set them to corresponding self.xxxx values
        pass

    def _start_ventilation(self):
        # open drain valves  'set_air_valves'
        self._set_new_relay_state('set_air_valves', 0)
        # wait time to open them ~15 sec
        rospy.sleep(self._air_valves_open_time)
        # start drain pumps 'set_air_pumps'
        self._set_new_relay_state('set_air_pumps', 0)
        pass

    def _stop_ventilation(self):
        # stop drain pumps
        self._set_new_relay_state('set_air_pumps', 1)
        # wait 1 sec
        rospy.sleep(1.0)
        # close drain valves
        self._set_new_relay_state('set_air_valves', 1)
        pass

    def _perform_sba5_calibration(self):
        # open n2 valve
        # send Z to sba-5
        # wait for calibration time 21/40/90 sec
        # close n2 valve
        pass

    def _publish_sba5_measure(self, data):
        co2_msg = Temperature()
        co2_msg.header.stamp = rospy.Time.now()
        co2_msg.temperature = data
        self._co2_pub.publish()

    def _set_new_relay_state(self, command, state):
        # command - str, state - 0 or 1
        self._logger.debug("start setting new relay mode '{}' '{}' ".format(command, state))
        rospy.wait_for_service(self._relay_service_name)
        try:
            relay_wrapper = rospy.ServiceProxy(self._relay_service_name, RelayDevice)
            resp = relay_wrapper(command, state)
            self._logger.debug(resp)
            return resp

        except rospy.ServiceException, e:
            self._logger.error("Service call failed: {}".format(e))

    def _set_new_light_mode(self, red, white):
        self._logger.debug("start settting new light mode {} {}".format(red, white))
        rospy.wait_for_service(self._led_service_name)
        try:
            led_wrapper = rospy.ServiceProxy(self._led_service_name, LedDevice)
            command = 'full_reconfigure'
            resp = led_wrapper(command, red, white)
            self._logger.debug(resp)
            return resp

        except rospy.ServiceException, e:
            self._logger.error("Service call failed: {}".format(e))

    def _get_sba5_measure(self, event=None):

        if self._sba5_measure_allowed:
            # event is rospy.TimerEvent
            rospy.wait_for_service(self._sba5_service_name)
            try:
                sba_device = rospy.ServiceProxy(self._sba5_service_name, SBA5Device)
                raw_resp = sba_device("measure_co2")
                self._logger.debug("We got raw response from sba5 {}".format(raw_resp))

                # lets get co2 measure from that string
                pattern = re.compile(r'\w+: (\d+.\d+)')  # for answers like "success: 55.21"
                co2_data = float(pattern.findall(raw_resp)[0])

                # add to self array
                self._add_new_data_to_array(co2_data)

                # then publish it
                self._publish_sba5_measure(co2_data)

                return float(co2_data)

            except Exception as e:
                self._logger.error("Service call failed: {}".format(e))

    def _add_new_data_to_array(self, data):
        pass

    def _handle_request(self, reqv):
        pass


if __name__ == "__main__":
    ControlSystemServer()
