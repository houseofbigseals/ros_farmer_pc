#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial
import re
import traceback, sys
import time
import numpy


class Scales:
    """
    simple wrapper for serial scales device
    just read and parse strings from selected serial
    """
    def __init__(self, dev=False, baud=False, timeout=False):
        self._dev = dev if dev else '/dev/serial/by-id/usb-USB_Vir_USB_Virtual_COM-if00'
        self._baud = baud if baud else 9600
        self._timeout = timeout if timeout else 1
        #self._scales = serial.Serial(self._dev, self._baud, timeout=self._timeout)

    def get_raw_data(self):
        with serial.Serial(self._dev, self._baud, timeout=self._timeout) as scales:
            try:
                raw_data = scales.readline()
                # pattern = re.compile(r'\w\w, \w\w, (\d+.\d+) \w')  # for answers like "ST, GS, 55.210 g"
                # w_data = float(pattern.findall(raw_data.response)[0])
                return raw_data
            except Exception as e:
                exc_info = sys.exc_info()
                err_list = traceback.format_exception(*exc_info)
                print("Service call failed: {}".format(err_list))
                return err_list

    def get_data(self):
        with serial.Serial(self._dev, self._baud, timeout=self._timeout) as scales:
            try:
                raw_data = scales.readline()
                pattern = re.compile(r'\w\w, \w\w, (\d+.\d+) \w')  # for answers like "ST, GS, 55.210 g"
                w_data = float(pattern.findall(raw_data)[0])
                return w_data
            except Exception as e:
                exc_info = sys.exc_info()
                err_list = traceback.format_exception(*exc_info)
                print("Service call failed: {}".format(err_list))
                return -66536.65

    def get_mean_data(self, dtime):
        w_datas = []
        t_start = time.time()
        with serial.Serial(self._dev, self._baud, timeout=self._timeout) as scales:
            while time.time() - t_start < dtime:
                try:
                    raw_data = scales.readline()
                    pattern = re.compile(r'\w\w,\w\w,\s*(\d+.\d+)\s*\w')  # for answers like "ST, GS, 55.210 g"
                    w_data = float(pattern.findall(raw_data)[0])
                    w_datas.append(w_data)
                except Exception as e:
                    exc_info = sys.exc_info()
                    err_list = traceback.format_exception(*exc_info)
                    print("Service call failed: {}".format(err_list))
        res = numpy.mean(w_datas)
        return res



if __name__ == "__main__":
    sc = Scales()
    print(sc.get_raw_data())
    print(sc.get_mean_data(10))
