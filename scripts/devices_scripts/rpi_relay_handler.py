#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
RPI gpio relay handler, for working withh relay directly

you have to connect relay module such as in that video
https://medium.com/@jinky32/connecting-a-12v-8-channel-relay-to-an-external-power-supply-and-raspberrypi-6fec119c112c
because we are using 12dc external pover supply
do not connect rpi GND with relay GND !

default pinout:

RELAY JD_VCC -> external 12VDC+
RELAY GND -> external 12VDC-
RELAY VCC -> RPI 3.3V

RELAY CH1 -> RPI GPIO5
RELAY CH2 -> RPI GPIO6
RELAY CH3 -> RPI GPIO12
RELAY CH4 -> RPI GPIO13
RELAY CH5 -> RPI GPIO19
RELAY CH6 -> RPI GPIO26
RELAY CH7 -> RPI GPIO16
RELAY CH8 -> RPI GPIO20

it works somehow and i dont know how exactly

"""


from copy import deepcopy
import time
from collections import OrderedDict, namedtuple

try:
    # import gpiozero
    from gpiozero import LED
except Exception as e:
    print(e)

relay_ch = namedtuple("relay_ch", ["name", "ch_num", "gpio_pin", "led_device"])

# "N2_VALVE,AIR_PUMP_1,AIR_PUMP_2,COOLER_1,AIR_VALVE_1,AIR_VALVE_2,NDIR_PUMP,EMPTY"


class RelayHandler(object):
    """
    super stupid hardcoded thing just to fix project, because it is on fire
    USE ONLY INSIDE CONTROL_SYSTEM NODE
    """
    def __init__(
            self,
            n2_valve=5,
            air_pump_1=6,
            air_pump_2=12,
            cooler_1=13,
            air_valve_1=19,
            air_valve_2=26,
            ndir_pump=16,
            empty=20
    ):
        """
        name: gpio_pin
        """
        # lets init all pins as low
        self.n2_valve = LED(n2_valve)
        self.air_pump_1 = LED(air_pump_1)
        self.air_pump_2 = LED(air_pump_2)
        self.cooler_1 = LED(cooler_1)
        self.air_valve_1 = LED(air_valve_1)
        self.air_valve_2 = LED(air_valve_2)
        self.ndir_pump = LED(ndir_pump)
        self.empty = LED(empty)

        # self.last_state =
        # lets set default value on relay:
        self.set_new_state(1, 1, 1, 1, 1, 1, 1, 0)

    def set_new_state(self,
                      n2_valve,
                      air_pump_1,
                      air_pump_2,
                      cooler_1,
                      air_valve_1,
                      air_valve_2,
                      ndir_pump,
                      empty
                      ):

        self.n2_valve.value = n2_valve
        self.air_pump_1.value = air_pump_1
        self.air_pump_2.value = air_pump_2
        self.cooler_1.value = cooler_1
        self.air_valve_1.value = air_valve_1
        self.air_valve_2.value = air_valve_2
        self.ndir_pump.value = ndir_pump
        self.empty.value = empty



    def get_states_str(self):
        return OrderedDict(
            [("n2_valve", self.n2_valve.value),
            ("air_pump_1", self.air_pump_1.value),
            ("air_pump_2", self.air_pump_2.value),
            ("cooler_1", self.cooler_1.value),
             ("air_valve_1", self.air_valve_1.value),
             ("air_valve_2", self.air_valve_2.value),
             ("ndir_pump", self.ndir_pump.value),
             ("empty", self.empty.value)]
        )

    def parse_old_command(self, command, arg):

        if command == 'shutdown':
            self.set_new_state(1, 1, 1, 1, 1, 1, 1, 1)
            return self.get_states_str()

        elif command == 'set_ndir_pump':
            self.ndir_pump.value = arg
            return self.get_states_str()

        elif command == 'set_vent_coolers':
            self.cooler_1.value = arg
            return self.get_states_str()

        elif command == 'set_n2_valve':
            self.n2_valve.value = arg
            return self.get_states_str()

        elif command == 'set_air_valves':
            self.air_valve_1.value = arg
            self.air_valve_2.value = arg
            return self.get_states_str()

        elif command == 'set_air_pumps':
            self.air_pump_1.value = arg
            self.air_pump_2.value = arg
            return self.get_states_str()

        elif command == 'respawn':
            # only for compatibility
            return self.get_states_str()


if __name__ == "__main__":
    # simple test
    rh = RelayHandler(
        n2_valve=5,
        air_pump_1=6,
        air_pump_2=12,
        cooler_1=13,
        air_valve_1=19,
        air_valve_2=26,
        ndir_pump=16,
        empty=20
    )

    while(True):
        rh.set_new_state(
            0, 1, 0, 1, 0, 1, 0, 1
        )
        print(rh.get_states_str())
        time.sleep(1)
        rh.set_new_state(
            0, 0, 0, 0, 0, 0, 0, 0
        )
        print(rh.get_states_str())
        time.sleep(1)