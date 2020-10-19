#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
RPI gpio relay handler, for working withh relay directly
"""
from copy import deepcopy
import time

try:
    # import gpiozero
    from gpiozero import LED
except Exception as e:
    print(e)


# "N2_VALVE,AIR_PUMP_1,AIR_PUMP_2,COOLER_1,AIR_VALVE_1,AIR_VALVE_2,NDIR_PUMP,EMPTY"

class RelayHandler(object):
    """
    super stupid hardcoded thing just to fix project, because it is on fire
    USE ONLY INSIDE CONTROL_SYSTEM NODE
    """
    def __init__(
            self,
            n2_valve,
            air_pump_1,
            air_pump_2,
            cooler_1,
            air_valve_1,
            air_valve_2,
            ndir_pump,
            empty
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

    def get_states(self):
        return {
            "n2_valve": self.n2_valve.value,
            "air_pump_1": self.air_pump_1.value,
            "air_pump_2": self.air_pump_2.value,
            "cooler_1": self.cooler_1.value,
            "air_valve_1": self.air_valve_1.value,
            "air_valve_2": self.air_valve_2.value,
            "ndir_pump": self.ndir_pump.value,
            "empty": self.empty.value,
                }


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
        print(rh.get_states())
        time.sleep(1)
        rh.set_new_state(
            0, 0, 0, 0, 0, 0, 0, 0
        )
        print(rh.get_states())
        time.sleep(1)