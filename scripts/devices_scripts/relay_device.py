#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray
from data_scripts.custom_logger import CustomLogger
from ros_farmer_pc.srv import RelayDevice
from future import *

# custom errors

class RelayDeviceException(Exception):
    pass

# # simple subclass
# class channel(object):
#
#     def __init__(self, name, pin, state):
#         self._pin = pin
#         self._name = name
#         self._state = state

class RelayDeviceServer(object):
    """
    class to receive messages from other ros nodes, handle it
    send messages to Relay driver and parse its output back as response

    - must receive a number of channels
    - must receive a string with comma separated list of object names.
    No spaces, only commas.
    If objects have same type, they must have the same name ending _number.
    Device on channel N in relay must be on N`s place in that list.
    If there no device on some channel, there must be keyword EMPTY.
    Example for 8-channel relay:
    "N2_VALVE_1,AIR_PUMP_1,AIR_PUMP_2,COOLER_1,AIR_VALVE_1,AIR_VALVE_2,EMPTY,EMPTY"
    """
    def __init__(self):

        # hardcoded constants
        self._success_response = "success"
        self._error_response = "error: "
        self._empty_key = 'EMPTY'
        self._cooler_key = 'COOLER'
        self._air_pump_key = 'AIR_PUMP'
        self._air_valve_key = 'AIR_VALVE'
        self._n2_valve_key = 'N2_VALVE'
        self._default_channel_state = 1



        # start node
        rospy.init_node('relay_device_server')

        # get roslaunch params and reinit part of params
        self._logname = rospy.get_param('~relay_log_name', 'relay')
        self._log_node_name = rospy.get_param('~relay_log_node_name', 'relay_log_node')
        self._service_name = rospy.get_param('~relay_service_name', 'relay_device')
        self._number_of_channels = rospy.get_param('~relay_number_of_channels', 8)
        self._mapping_string = rospy.get_param(
            '~relay_mapping_string',
            'N2_VALVE,AIR_PUMP_1,AIR_PUMP_2,COOLER_1,AIR_VALVE_1,AIR_VALVE_2,EMPTY,EMPTY')
        self._raw_relay_topic = rospy.get_param('~raw_relay_topic_name', 'relay_1_sub')

        self._commands = ['set_air_pumps', 'set_air_valves', 'set_n2_valve', 'set_vent_coolers']

        # create dictionary with
        # {key - devname: str, value - [number of channel:int, state:int]}
        devnames = self._mapping_string.split(',')
        self._map = dict()
        for i in range (0, len(devnames)):
            self._map.update({devnames[i]:[i, self._default_channel_state]})

        # create log topic publisher
        self._log_pub = rospy.Publisher(self._log_node_name, String, queue_size=10)

        # create raw_relay control channel publisher
        self._raw_relay_pub = rospy.Publisher(self._raw_relay_topic, Int16MultiArray, queue_size=10)

        # logger
        self._logger = CustomLogger(name=self._logname, logpub=self._log_pub)
        self._logger.debug("relay_device_server init")

        # service
        self._service = rospy.Service(self._service_name, RelayDevice, self.handle_request)
        self._loop()

    def _loop(self):
        rospy.spin()

    def handle_request(self, req):
        # check params from request
        # at first find what user wants from us:
        self._logger.debug("+++++++++++++++++++++++++++++++++++++++++++++++")
        self._logger.debug("+++++++++++++++++++++++++++++++++++++++++++++++")
        self._logger.debug("we got request: {}".format(req))

        if not req.command:
            #if we got empty string
            resp = self._error_response + 'empty_command'
            return resp

        elif req.command == 'set_air_pumps':
            try:
                self._set_air_pumps(req.state)
                resp = self._success_response
                return resp
            except RelayDeviceException as e:
                resp = self._error_response + e.args[0]
                return resp

        elif req.command == 'set_air_valves':
            try:
                self._set_air_valves(req.state)
                resp = self._success_response
                return resp
            except RelayDeviceException as e:
                resp = self._error_response + e.args[0]
                return resp

        elif req.command == 'set_n2_valve':
            try:
                self._set_n2_valve(req.state)
                resp = self._success_response
                return resp
            except RelayDeviceException as e:
                resp = self._error_response + e.args[0]
                return resp

        elif req.command == 'set_vent_coolers':
            try:
                self._set_vent_coolers(req.state)
                resp = self._success_response
                return resp
            except RelayDeviceException as e:
                resp = self._error_response + e.args[0]
                return resp

        elif req.command == 'help':
            resp = "service commands is {}".format(self._commands)+\
                "0 - to activate relay channel, 1 to close \n" + "map of devices and channels: \n"+\
                "{key - device name: str, value - [number of channel: int, state: int]}"\
                   + "\n" + str(self._map)
            return resp

        else:
            resp = self._error_response + 'unknown command'
            return resp

        # ==================== real commands ==================

    def _set_raw_relay_pins(self, array):

        # This method dont handle self._current_state
        if len(array) == 0 or len(array) > self._number_of_channels:
            raise RelayDeviceException(" length of new relay state array is out of range ")
        else:
            # 0 is enable relay
            # else is disable relay
            # because all our relay devices use inverse logic

            # this method wraps the low-level call of real relay device through serial node
            # for now it is topic
            try:
                # create msg
                new_msg = Int16MultiArray()
                new_msg.data = array

                self._raw_relay_pub.publish(new_msg)
                self._logger.debug("publish new raw relay state {}".format(array))

            except Exception as e:
                raise RelayDeviceException("Error while publish new state message {}".format(e))

    def _set_pin_by_name(self, name, state):
        if name not in self._map.keys():
            raise RelayDeviceException("cant find device name in list of devices, try help")
        elif state != 0 and state != 1:
            raise RelayDeviceException("wrong type of new state, state must be 0 or 1")
        else:
            # 0 is enable relay
            # else is disable relay
            # because all our relay devices use inverse logic

            # update remembered state and add here new pin state
            # change state
            self._map[name][1] = state
            self._logger.debug(str(self._map))

            # get list of sorted values by pin number
            sort_  = list(sorted(self._map.values(), key=lambda t: t[0]))
            self._logger.debug(str(sort_))

            # then get from sorted array only states
            pin_states_ = list( x[1] for x in sort_)
            self._logger.debug(str(pin_states_))

            # then send it to raw relay sub
            self._set_raw_relay_pins(pin_states_)

    def _set_group_pins_by_key(self, key, state):
        if state != 0 and state != 1:
            raise RelayDeviceException("wrong type of new state, state must be 0 or 1")
        else:
            # find all keys in  self._map those hold key in prefix
            selected_keys = list(n for n in self._map.keys() if key in n)
            # change state
            for name in selected_keys:
                self._map[name][1] = state
            self._logger.debug(str(self._map))

            # get list of sorted values by pin number
            sort_  = list(sorted(self._map.values(), key=lambda t: t[0]))
            self._logger.debug(str(sort_))

            # then get from sorted array only states
            pin_states_ = list(x[1] for x in sort_)
            self._logger.debug(str(pin_states_))

            # then send it to raw relay sub
            self._set_raw_relay_pins(pin_states_)

    def _set_air_pumps(self, state):
        self._set_group_pins_by_key(self._air_pump_key, state)

    def _set_air_valves(self, state):
        self._set_group_pins_by_key(self._air_valve_key, state)

    def _set_n2_valve(self, state):
        self._set_group_pins_by_key(self._n2_valve_key, state)

    def _set_vent_coolers(self, state):
        self._set_group_pins_by_key(self._cooler_key, state)


if __name__ == "__main__":
    RelayDeviceServer()

