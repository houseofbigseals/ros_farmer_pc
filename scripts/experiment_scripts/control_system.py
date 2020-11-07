#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from std_msgs.msg import String
import traceback
import datetime
import sys
from custom_logger import CustomLogger
from ros_farmer_pc.srv import ControlSystem, LedDevice, RelayDevice, SBA5Device, ExpSystem, ExpSystemResponse
from sensor_msgs.msg import Temperature
from threading import Lock, Event
import time
import re
import os
from rosgraph_msgs.msg import Log
import roslaunch
from rpi_relay_handler import RelayHandler
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
        self._logsep = ";"

        # start node
        rospy.init_node('control_server', log_level=rospy.DEBUG)

        # get roslaunch params

        # names for self topics
        self._logname = rospy.get_param('~control_log_name', 'control_system')
        self._log_node_name = rospy.get_param('~control_log_node_name', 'control_log_node')
        self._service_name = rospy.get_param('~control_service_name', 'control_system')
        self._raw_co2_pub_name = rospy.get_param('~control_raw_co2_pub_name', 'raw_co2_pub')
        self._operator_pub_name = rospy.get_param('~control_operator_pub_name', 'operator_pub')

        # devices services names
        # self._relay_service_name = rospy.get_param('~control_relay_service_name', 'relay_device')
        self._led_service_name = rospy.get_param('~control_led_service_name', 'led_device')
        self._sba5_service_name = rospy.get_param('~control_sba5_service_name', 'sba5_device')  # we are using sba5 by default co2 sensor
        self._exp_service_name = rospy.get_param('~control_exp_service_name', 'exp_system')

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
        # search_experiment
        # ...
        #self._sba5_measure_allowed = False  # by default
        self._at_work = rospy.get_param('~control_start_at_work', True)
        # start node and loop immediately after launch or

        # rpi_gpio device stub
        # TODO add this params to .launch file
        self._gpio_handler = RelayHandler(
            n2_valve=5,
            air_pump_1=6,
            air_pump_2=12,
            cooler_1=13,
            air_valve_1=19,
            air_valve_2=26,
            ndir_pump=16,
            empty=20
        )


        # params of search experiment
        self._co2_search_time_start = 0
        self._co2_search_time_stop = 0
        self._current_search_point_id = 0
        self._current_red = self._default_red
        self._current_white = self._default_white
        self._current_search_is_finished = 1  # by default it is not finished



        # create log topic publisher
        self._log_pub = rospy.Publisher(self._log_node_name, String, queue_size=10)
        # create data topic publisher
        self._co2_pub = rospy.Publisher(self._raw_co2_pub_name, Temperature, queue_size=10)
        # create operator-topic to send important msgs to operator
        self._operator_pub = rospy.Publisher(self._operator_pub_name, String, queue_size=10)

        # logger
        self._logger = CustomLogger(name=self._logname, logpub=self._log_pub)
        self._logger.info("control_server init")

        # create Locks
        self._sba5_measure_allowed_event = Event()

        # create timers for async periodic tasks using internal ros mechanics
        # Create a ROS Timer for reading data
        rospy.Timer(rospy.Duration(1.0), self._get_sba5_measure)  # 1 Hz
        # create ros timer for main loop

        self._logger.info("control_server service creation")

        # service
        self._service = rospy.Service(self._service_name, ControlSystem, self._handle_request)

        self._logger.info("check if we are in experiment mode")


        self._serial_error_counter = 0
        self._serial_error_max = 5
        # subscribe to rosout_agg to know if there is fatal error and we need to do smth
        self._system_log_sub = rospy.Subscriber(
            name='/rosout_agg', data_class=Log,
            callback=self._log_callback,
            queue_size=5)

        # reinit sba5
        if self._mode == 'experiment' or self._mode == 'full_experiment':
            self._logger.info("update sba5 and allow sba5 measures")
            self._update_sba5_params()

            # allow measures of sba5
            self._sba5_measure_allowed_event.set()

            self._default_red = 120
            self._default_white = 120

            self._current_red = self._default_red
            self._current_white = self._default_white

        self._logger.info("go to loop")
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
            elif self._mode == 'full_experiment':
                self._full_experiment_loop()
            else:
                self._logger.error("current mode is not real mode: {}".format(self._mode))
                rospy.sleep(1)
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
            # self._set_new_relay_state('set_ndir_pump', 0)  # for test only
            # wait for self._ventilation_time
            rospy.sleep(self._ventilation_time)  # its not bad because service calls works in parallel threads
            # then stop it
            self._stop_ventilation()
            # self._set_new_relay_state('set_ndir_pump', 1)  # for test only
            # then wait and do nothing


    def _experiment_loop(self):
        # one loop
        # all experiment
        t = time.localtime()
        # every 15 minutes by default
        if t.tm_min % (self._full_experiment_loop_time/60.0) == 0:
            # start it again
            self._logger.info("start experiment loop again")
            # get new regime
            self._update_control_params()
            # set inside ventilation coolers on
            self._set_new_relay_state('set_vent_coolers', 0)
            # start ventilation and calibration
            self._start_ventilation()
            # stop measuring co2 using threading event
            self._sba5_measure_allowed_event.clear()
            self._logger.info("We have set measure flag to {}".format(self._sba5_measure_allowed_event.is_set()))
            # do calibration of sba-5
            self._perform_sba5_calibration()
            # start measuring co2 again
            self._sba5_measure_allowed_event.set()
            self._logger.info("We have set measure flag to {}".format(self._sba5_measure_allowed_event.is_set()))
            # wait for self._ventilation_time
            rospy.sleep(self._ventilation_time)
            # stop ventilation
            self._stop_ventilation()
            # wait self._isolated_measure_time
            rospy.sleep(self._isolated_measure_time)

    def _full_experiment_loop(self):
        # one loop
        # all experiment
        t = time.localtime()
        # every 15 minutes by default
        if t.tm_min % (self._full_experiment_loop_time/60.0) == 0:
            # start it again
            self._logger.info("start full experiment loop again")
            self._operator_call("start full experiment loop again")
            # self._get_current_point()

            # set inside ventilation coolers on
            self._set_new_relay_state('set_vent_coolers', 0)
            # start ventilation and calibration
            self._start_ventilation()
            self._operator_call("ventilation started")
            # get new led light params from exp_node
            self._get_current_point()
            self._logger.info("we got new exp point: id={} red={} white={}".format(
                self._current_search_point_id, self._current_red, self._current_white
            ))
            self._operator_call("we got new exp point: id={} red={} white={}".format(
                self._current_search_point_id, self._current_red, self._current_white
            ))
            self._set_new_light_mode(self._current_red, self._current_white)
            # stop measuring co2 using threading event
            self._sba5_measure_allowed_event.clear()
            self._logger.info("We have set measure flag to {}".format(self._sba5_measure_allowed_event.is_set()))
            # do calibration of sba-5
            self._operator_call("sba5 calibration started")
            self._perform_sba5_calibration()
            self._operator_call("sba5 calibration ended")
            # start measuring co2 again
            self._sba5_measure_allowed_event.set()
            self._logger.info("We have set measure flag to {}".format(self._sba5_measure_allowed_event.is_set()))
            #
            # self._co2_search_time_start = rospy.Time.now()
            # # send sign to operator
            # self._operator_call("co2_search_time started {}".format(self._co2_search_time_start))

            # wait for self._ventilation_time
            rospy.sleep(self._ventilation_time)
            # stop ventilation
            self._stop_ventilation()
            self._operator_call("stop ventilation")

            self._co2_search_time_start = rospy.Time.now()
            # send sign to operator

            ts = datetime.datetime.fromtimestamp(
                self._co2_search_time_start.to_sec()).strftime('%Y_%m_%d %H:%M:%S')
            self._operator_call("co2_search_time started {}".format(ts))
            self._logger.info("co2_search_time started {}".format(ts))
            # wait self._isolated_measure_time
            rospy.sleep(self._isolated_measure_time)

            self._co2_search_time_stop = rospy.Time.now()

            te = datetime.datetime.fromtimestamp(
                self._co2_search_time_start.to_sec()).strftime('%Y_%m_%d %H:%M:%S')

            self._operator_call("co2_search_time stopped {}".format(te))
            self._logger.info("co2_search_time stopped {}".format(te))
            # send start and stop times of this search point to exp_node
            self._send_point_data()
            self._operator_call("data sent to exp_system")

    # =============================== support methods ==============================

    def _operator_call(self, msg):
        # just sends msg to operator topic
        _time = datetime.datetime.now()
        msg = str(_time) + self._logsep + str(self._logname) + self._logsep + msg
        self._operator_pub.publish(msg)

    def _log_callback(self, log_msg):

        # hardcoded thing just to handle serial errors

        if log_msg.name == '/serial_node' and log_msg.level == 8:
            self._logger.error("we got serial error {}".format(log_msg))
            self._serial_error_counter += 1
            if self._serial_error_counter >= self._serial_error_max:
                self._logger.error("max number of serial error counted: {}".format(self._serial_error_counter))
                self._restart_serial_node()

                self._serial_error_counter = 0


    def _restart_serial_node(self):
        # we just need to kill serial node
        # master must rebirth it

        # self._logger.warning("trying to kill relay node")

        # os.system("rosnode kill /relay_device")
        # time.sleep(1.5)

        self._logger.warning("trying to kill serial node")

        os.system("rosnode kill /serial_node")

        self._logger.warning("waiting for serial node respawn")

        time.sleep(1.5)

        # then check if it was created again
        serial_node_found = False
        # relay_device_found = False
        # while not serial_node_found and relay_device_found:
        while not serial_node_found:
            nodes = os.popen("rosnode list").readlines()
            for i in range(len(nodes)):
                nodes[i] = nodes[i].replace("\n", "")
                if nodes[i] == "/serial_node":
                    serial_node_found = True
                    self._logger.warning("found serial node in rosnode list")

                # if nodes[i] == "/relay_device":
                #     relay_device_found = True
                #     self._logger.warning("found relay_device in rosnode list")
            time.sleep(0.5)

        # time.sleep(5)

        # # then send respawn signal to relay device to restore relay state on mcu
        # self._logger.warning("send respawn signal to relay")
        # rospy.wait_for_service(self._relay_service_name)
        # try:
        #     relay_wrapper = rospy.ServiceProxy(self._relay_service_name, RelayDevice)
        #     resp = relay_wrapper("respawn", 1)
        #     self._logger.debug(resp)
        #     return resp
        #
        # except rospy.ServiceException, e:
        #     exc_info = sys.exc_info()
        #     err_list = traceback.format_exception(*exc_info)
        #     self._logger.error("Service call failed: {}".format(err_list))


    def _send_point_data(self):
        self._logger.info("try to send data about current search point")
        rospy.wait_for_service(self._exp_service_name)
        try:
            exp_device = rospy.ServiceProxy(self._exp_service_name, ExpSystem)
            resp = exp_device(command="set_point_data",
                              point_id=self._current_search_point_id,
                              start_time=self._co2_search_time_start,
                              end_time=self._co2_search_time_stop
                              )
            self._logger.debug(resp)
            return resp

        except Exception as e:
            exc_info = sys.exc_info()
            err_list = traceback.format_exception(*exc_info)
            self._logger.error("Service call failed: {}".format(err_list))

    def _get_current_point(self):
        self._logger.info("try to get new search point")
        rospy.wait_for_service(self._exp_service_name, 1) # NOTE check this place
        try:
            exp_device = rospy.ServiceProxy(self._exp_service_name, ExpSystem)
            resp = exp_device(command="get_current_point")
            self._current_search_point_id = resp.point_id
            self._current_red = resp.red
            self._current_white = resp.white
            self._logger.info(resp)
            return resp

        except Exception as e:
            exc_info = sys.exc_info()
            err_list = traceback.format_exception(*exc_info)
            self._logger.error("Service call failed: {}".format(err_list))

    def _update_control_params(self):
        # get new params and set them to corresponding self.xxxx values
        # NOTE: method is DEPRECATED
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

        self._logger.debug("start sba recalibration")
        # stop NDIRGA external air pump
        self._set_new_relay_state('set_ndir_pump', 1)
        # open n2 valve
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
            exc_info = sys.exc_info()
            err_list = traceback.format_exception(*exc_info)
            self._logger.error("Service call failed: {}".format(err_list))
            # raise ControlSystemException(e)

        # close n2 valve
        self._set_new_relay_state('set_n2_valve', 1)
        # start NDIRGA external air pump
        self._set_new_relay_state('set_ndir_pump', 0)
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
        self._logger.info("start setting new relay mode '{}' '{}' ".format(command, state))
        # rospy.wait_for_service(self._relay_service_name)
        try:
        #     relay_wrapper = rospy.ServiceProxy(self._relay_service_name, RelayDevice)
        #     resp = relay_wrapper(command, state)
        #     self._logger.debug(resp)
        #     return resp

            self._gpio_handler.parse_old_command(command=command, arg=state)

        except Exception, e:
            exc_info = sys.exc_info()
            err_list = traceback.format_exception(*exc_info)
            self._logger.error("Service call failed: {}".format(err_list))

            # self._logger.error("Service call failed: {}".format(e))
            # raise ControlSystemException(e)

    def _set_new_light_mode(self, red, white):
        self._logger.info("start setting new light mode {} {}".format(red, white))
        rospy.wait_for_service(self._led_service_name)
        try:
            led_wrapper = rospy.ServiceProxy(self._led_service_name, LedDevice)
            command = 'full_reconfigure'
            resp = led_wrapper(command, red, white)
            self._logger.info(resp)
            return resp

        except rospy.ServiceException, e:
            exc_info = sys.exc_info()
            err_list = traceback.format_exception(*exc_info)
            self._logger.error("Service call failed: {}".format(err_list))
            # self._logger.error("Service call failed: {}".format(e))
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
            # self._add_new_data_to_array(co2_data)

            # then publish it
            self._publish_sba5_measure(co2_data)
            #self._logger.debug("We published it")

            return float(co2_data)

        except Exception as e:
            exc_info = sys.exc_info()
            err_list = traceback.format_exception(*exc_info)
            self._logger.warning("Service call failed: {}".format(err_list))

            # print("Service call failed: {}".format(e))
            # self._logger.error("Service call failed: {}".format(e))
            #raise ControlSystemException(e)
            #raise

    def _update_sba5_params(self):
        self._logger.info("Try to reinit sba5")
        rospy.wait_for_service(self._sba5_service_name)
        try:
            sba_device = rospy.ServiceProxy(self._sba5_service_name, SBA5Device)
            raw_resp = sba_device('init')
            self._logger.info("We got raw response from sba5 {}".format(raw_resp))
            return raw_resp

        except Exception as e:
            exc_info = sys.exc_info()
            err_list = traceback.format_exception(*exc_info)
            self._logger.error("Service call failed: {}".format(err_list))
            # self._logger.error("Service call failed: {}".format(e))
            raise ControlSystemException(e)


    def _shutdown(self):

        # set mode as test
        self._mode = 'test'

        self._logger.debug("trying to shutdown all relay things")
        # try to kill all relay-connected devices
        # dont care if there is SBA5 calibration

        self._set_new_relay_state('shutdown', 1)


        # self._set_new_relay_state('set_air_pumps', 1)
        # self._set_new_relay_state('set_air_valves', 1)
        # self._set_new_relay_state('set_n2_valve', 1)
        # self._set_new_relay_state('set_vent_coolers', 1)
        # self._set_new_relay_state('set_ndir_pump', 1)

        # kill led lights to minimum light

        self._logger.info("start stopping led lamps")
        rospy.wait_for_service(self._led_service_name)
        try:
            led_wrapper = rospy.ServiceProxy(self._led_service_name, LedDevice)
            command = 'stop'
            resp = led_wrapper(command, 10, 10)
            self._logger.info(resp)
            return resp

        except rospy.ServiceException, e:
            exc_info = sys.exc_info()
            err_list = traceback.format_exception(*exc_info)
            self._logger.error("Service call failed: {}".format(err_list))
            # self._logger.error("Service call failed: {}".format(e))
        pass

    def _handle_request(self, req):
        self._logger.debug("+++++++++++++++++++++++++++++++++++++++++++++++")
        self._logger.debug("+++++++++++++++++++++++++++++++++++++++++++++++")
        self._logger.info("we got request: {}".format(req))

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

        elif req.command == 'shutdown':
            try:
                self._shutdown()
                resp = self._success_response
                return resp
            except Exception as e:  # check, if it really need to catch here all exceptions?
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
                resp = self._success_response + "mode was set to " + self._mode
            elif req.argument == 'life_support':
                self._mode = 'life_support'
                resp = self._success_response + "mode was set to " + self._mode
            elif req.argument == 'test':
                self._mode = 'test'
                resp = self._success_response + "mode was set to " + self._mode
            else:
                resp = self._error_response + "no such control mode"

            return resp

        elif req.command == 'get_mode':
            try:
                # self._update_sba5_params()
                resp = self._success_response + " current mode is " + self._mode + " current red is " +\
                       str(self._current_red) + ' current white is ' + str(self._current_white)
                return resp
            except ControlSystemException as e:
                resp = self._error_response + e.args[0]
                return resp

        else:
            resp = self._error_response + 'unknown command'
            return resp



if __name__ == "__main__":
    ControlSystemServer()
