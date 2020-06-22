#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from std_msgs.msg import String
from data_scripts.custom_logger import CustomLogger
from ros_farmer_pc.srv import ControlSystem, LedDevice, RelayDevice, SBA5Device, SBA5DeviceResponse
from sensor_msgs.msg import Temperature
from threading import Lock, Event
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
        self._n2_calibration_time = rospy.get_param('~control_n2_calibration_time', 90.0)  # depends on
        self._air_valves_open_time = rospy.get_param('~control_air_valves_open_time', 15.0)  # sec
        self._co2_measure_time = rospy.get_param('~control_co2_measure_time', 1.0)  # sec
        self._ventilation_time = self._full_experiment_loop_time - self._isolated_measure_time - \
            self._n2_calibration_time - self._air_valves_open_time

        # flags of current control regime
        #self._mode = rospy.get_param('~control_start_mode', 'life_support')
        self._mode = rospy.get_param('~control_start_mode', 'experiment')
        # mode can be :
        # experiment
        # life_support
        # test
        # ...
        #self._sba5_measure_allowed = False  # by default
        self._at_work = rospy.get_param('~control_start_at_work', True)
        # start node and loop immediately after launch or


        # create log topic publisher
        self._log_pub = rospy.Publisher(self._log_node_name, String, queue_size=10)
        # create data topic publisher
        self._co2_pub = rospy.Publisher(self._raw_co2_pub_name, Temperature, queue_size=10)

        # logger
        self._logger = CustomLogger(name=self._logname, logpub=self._log_pub)
        self._logger.debug("control_server init")

        # create Locks
        self._sba5_measure_allowed_event = Event()

        # create timers for async periodic tasks using internal ros mechanics
        # Create a ROS Timer for reading data
        rospy.Timer(rospy.Duration(2.0), self._get_sba5_measure)  # 2 Hz
        # create ros timer for main loop

        # TODO connect to led and relay services
        # or not

        # service
        self._service = rospy.Service(self._service_name, ControlSystem, self._handle_request)

        # reinit sba5
        self._update_sba5_params()
        # allow measures of sba5
        self._sba5_measure_allowed_event.set()

        self._loop()

    # ===================== loops for different control modes ======================

    def _loop(self):
        while not rospy.is_shutdown():
            # check if mode was changed
            if self._mode == 'experiment':
                self._experiment_loop()
            elif self._mode == 'life_support':
                self._life_support_loop()
            elif self._mode == 'test':
                self._test_loop()
            pass
            #rospy.spin()

    def _test_loop(self):
        t = time.localtime()
        # every 15 minutes by default
        if t.tm_min % (self._full_experiment_loop_time / 60.0) == 0:
            self._logger.debug("start test loop again")
            rospy.sleep(self._air_valves_open_time)
            self._logger.debug("we have done nothing, really")

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
            self._logger.debug("start experiment loop again")
            # set inside ventilation coolers on
            self._set_new_relay_state('set_vent_coolers', 0)
            # start ventilation and calibration
            self._start_ventilation()
            # stop measuring co2 using threading event
            self._sba5_measure_allowed_event.clear()
            self._logger.debug("We have set measure flag to {}".format(self._sba5_measure_allowed_event.is_set()))
            # do calibration of sba-5
            self._perform_sba5_calibration()
            # start measuring co2 again
            self._sba5_measure_allowed_event.set()
            self._logger.debug("We have set measure flag to {}".format(self._sba5_measure_allowed_event.is_set()))
            # wait for self._ventilation_time
            rospy.sleep(self._ventilation_time)
            # stop ventilation
            self._stop_ventilation()
            # wait self._isolated_measure_time
            rospy.sleep(self._isolated_measure_time)
            # get new regime
            self._update_control_params()

    # =============================== support methods ==============================

    def _update_control_params(self):
        # get new params and set them to corresponding self.xxxx values

        # differentiate all collected on current step data
        pass
        # send this data, current params and gotten F to G-calculation method
        pass
        # send all data and G to search method, to get new light mode params
        pass
        # set them
        self._set_new_light_mode(self._default_red, self._default_white)  # for a time

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
        self._logger.debug("start sba recalibration")
        self._set_new_relay_state('set_n2_valve', 0)
        # send Z to sba-5
        rospy.wait_for_service(self._sba5_service_name)
        try:
            sba_device = rospy.ServiceProxy(self._sba5_service_name, SBA5Device)
            raw_resp = sba_device('recalibrate')
            self._logger.debug("We got raw response from sba5 {}".format(raw_resp))
            # wait for calibration time 21/40/90 sec
            rospy.sleep(self._n2_calibration_time)

        except Exception as e:
            self._logger.error("Service call failed: {}".format(e))
            # raise ControlSystemException(e)

        # close n2 valve
        self._set_new_relay_state('set_n2_valve', 1)
        pass

    def _publish_sba5_measure(self, data):
        self._logger.debug("publish_sba5_measure: We got data: {}".format(data))
        co2_msg = Temperature()
        co2_msg.header.stamp = rospy.Time.now()
        co2_msg.temperature = data
        self._logger.debug("Message: {}".format(co2_msg))
        self._co2_pub.publish(co2_msg)

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
            # raise ControlSystemException(e)

    def _set_new_light_mode(self, red, white):
        self._logger.debug("start setting new light mode {} {}".format(red, white))
        rospy.wait_for_service(self._led_service_name)
        try:
            led_wrapper = rospy.ServiceProxy(self._led_service_name, LedDevice)
            command = 'full_reconfigure'
            resp = led_wrapper(command, red, white)
            self._logger.debug(resp)
            return resp

        except rospy.ServiceException, e:
            self._logger.error("Service call failed: {}".format(e))
            #raise ControlSystemException(e)

    def _get_sba5_measure(self, event=None):

        # if self._sba5_measure_allowed:
        # event is rospy.TimerEvent
        self._sba5_measure_allowed_event.wait()
        rospy.wait_for_service(self._sba5_service_name)
        try:
            sba_device = rospy.ServiceProxy(self._sba5_service_name, SBA5Device)
            raw_resp = sba_device("measure_co2")
            #self._logger.debug("We got raw response from sba5 : {}".format(raw_resp.response))
            #self._logger.debug("Type of raw response : {}".format(type(raw_resp.response)))
            #self._logger.debug("Size of raw response : {}".format(len(raw_resp.response)))
            # lets get co2 measure from that string
            pattern = re.compile(r'\w+: (\d+.\d+)')  # for answers like "success: 55.21"
            co2_data = float(pattern.findall(raw_resp.response)[0])

            self._logger.debug("measure_co2: We find co2 using re : {}".format(co2_data))
            # add to self array
            self._add_new_data_to_array(co2_data)

            # then publish it
            self._publish_sba5_measure(co2_data)
            #self._logger.debug("We published it")

            return float(co2_data)

        except Exception as e:
            print("Service call failed: {}".format(e))
            self._logger.error("Service call failed: {}".format(e))
            #raise ControlSystemException(e)
            #raise
            # TODO FIX

    def _update_sba5_params(self):
        self._logger.debug("Try to reinit sba5")
        rospy.wait_for_service(self._sba5_service_name)
        try:
            sba_device = rospy.ServiceProxy(self._sba5_service_name, SBA5Device)
            raw_resp = sba_device('init')
            self._logger.debug("We got raw response from sba5 {}".format(raw_resp))
            return raw_resp

        except Exception as e:
            self._logger.error("Service call failed: {}".format(e))
            raise ControlSystemException(e)



    def _add_new_data_to_array(self, data):
        pass

    def _handle_request(self, req):
        self._logger.debug("+++++++++++++++++++++++++++++++++++++++++++++++")
        self._logger.debug("+++++++++++++++++++++++++++++++++++++++++++++++")
        self._logger.debug("we got request: {}".format(req))

        if not req.command:
            # if we got empty string
            resp = self._error_response + 'empty_command'
            return resp

        elif req.command == 'perform_sba5_calibration':
            try:
                self._perform_sba5_calibration()
                resp = self._success_response
                return resp
            except ControlSystemException as e:
                resp = self._error_response + e.args[0]
                return resp

        elif req.command == 'start_ventilation':
            try:
                self._start_ventilation()
                resp = self._success_response
                return resp
            except ControlSystemException as e:
                resp = self._error_response + e.args[0]
                return resp

        elif req.command == 'stop_ventilation':
            try:
                self._stop_ventilation()
                resp = self._success_response
                return resp
            except ControlSystemException as e:
                resp = self._error_response + e.args[0]
                return resp

        elif req.command == 'update_sba5_parameters':
            try:
                self._update_sba5_params()
                resp = self._success_response
                return resp
            except ControlSystemException as e:
                resp = self._error_response + e.args[0]
                return resp

        elif req.command == 'get_co2_measure':
            if self._sba5_measure_allowed_event.is_set():
                try:
                    co2 = self._get_sba5_measure()
                    resp = self._success_response + str(co2)
                    return resp
                #except ControlSystemException as e:
                except Exception as e:
                    resp = self._error_response + str(e) #+e.args[0]
                    return resp
            else:
                resp = self._error_response + "co2 measures are not allowed now"
                return resp

        elif req.command == 'set_mode':
            if req.argument == 'experiment':
                self._mode = 'experiment'
                resp = self._success_response
            elif req.argument == 'life_support':
                self._mode = 'life_support'
                resp = self._success_response
            elif req.argument == 'test':
                self._mode = 'test'
                resp = self._success_response
            else:
                resp = self._error_response + "no such control mode"

            return resp

        else:
            resp = self._error_response + 'unknown command'
            return resp



if __name__ == "__main__":
    ControlSystemServer()
