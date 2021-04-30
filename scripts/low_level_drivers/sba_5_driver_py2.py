#!/usr/bin/env python
# -*- coding: utf-8 -*-


# this module contains methods to get data
# and send commands to SBA-5 CO2 Gas Analyzer
# from PP systems


import serial
import time
import csv
import logging

#logger = logging.getLogger("Worker.Units.CO2Sensor.SBA5Wrapper")
logging.basicConfig(filename='sba_5_driver.log',
                    format='%(asctime)s;%(levelname)s;%(message)s',
                    level=logging.DEBUG)

class SBAWrapper(object):
    """
    This class wraps all sba commands as a class methods or fields
    """
    def __init__(
            self,
            devname='/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DN03WQZS-if00-port0',
            baudrate=19200,
            timeout= 0.1
    ):
        self.dev = devname
        self.baud = baudrate
        self.timeout = timeout

    def send_command(self, command):
        """
        Command must ends with \r\n !
        Its important
        :param command:
        :return:
        """
        """
        To initiate a command, the USB port or RS232 port sends an ASCII character or string.
        A single character command is acted on immediately when the character is received.
        A string command is acted on after the command string terminator <CR> is received. 
        The command can be sent with or without a checksum. If a checksum is sent, a “C” follows 
        the checksum value.
        For example,
        Device sends command without checksum: S,11,1<CR>
        Device sends command with checksum: S,11,1,043C<CR>
        On successfully receiving a command string, the SBA5+ sends an acknowledgement by 
        echoing back to all the ports the Command String and “OK”, each terminated with a <CR> 
        and<linefeed>.
        """
        # \r\n
        try:
            ser = serial.Serial(
                port=self.dev,
                baudrate=self.baud,
                timeout=self.timeout
            )
            bcom = command.encode('utf-8')
            ser.write(bcom)
        except Exception as e:
            logging.error("SBAWrapper error while send command: {}".format(e))
        # then try to read answer
        # it must be two messages, ended with \r\n
        try:
            ser = serial.Serial(
                port=self.dev,
                baudrate=self.baud,
                timeout=self.timeout
            )
            echo = (ser.readline()).decode('utf-8')
            status = (ser.readline()).decode('utf-8')
            return echo+status
        except Exception as e:
            logging.error("SBAWrapper error while read answer from command: {}".format(e))

    def get_periodic_data(self):
        try:
            ser = serial.Serial(
                port=self.dev,
                baudrate=self.baud,
                timeout=self.timeout
            )
            ans = (ser.readline()).decode('utf-8')
            return ans
        except Exception as e:
            logging.error("SBAWrapper error while read: {}".format(e))


def read():
    ser = serial.Serial(
        port='/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DN03WQZS-if00-port0',
        baudrate=19200,
        timeout=10
    )
    while True:
        ans = (ser.readline()).decode('utf-8')
        print(ans)


def send_command(c):
    ser = serial.Serial(
        port='/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DN03WQZS-if00-port0',
        baudrate=19200,
        timeout=10
    )
    ser.write(c)
    ans = (ser.readline()).decode('utf-8')
    print(ans)
    ans = (ser.readline()).decode('utf-8')
    print(ans)
    while True:
        ans = (ser.readline()).decode('utf-8')
        print(ans)


def read_loop():
    s = SBAWrapper()
    date_ = time.strftime("%x", time.localtime())
    time_ = time.strftime("%X", time.localtime())
    _datafile = 'co2_test_{}.csv'.format(time.time())

    pump_on = s.send_command('P1\r\n')
    print(pump_on)

    while True:
        co2 = s.send_command('M\r\n')
        fieldnames = ["date", "time", "CO2"]
        date_ = time.strftime("%x", time.localtime())
        time_ = time.strftime("%X", time.localtime())
        data = {
            "date": date_,
            "time": time_,
            "CO2": co2.split(' ')[3],
        }

        with open(_datafile, "a") as out_file:
            writer = csv.DictWriter(out_file, delimiter=',', fieldnames=fieldnames)
            writer.writerow(data)

        print(co2)
        time.sleep(2)


def test():
    s = SBAWrapper()
    co2 = s.send_command('M\r\n')
    print(co2)
    pump_off = s.send_command('P0\r\n')
    print(pump_off)


def test_calibration():
    s = SBAWrapper()
    #pump_off = s.send_command('P0\r\n')
    #print(pump_off)
    print(s.send_command("EL\r\n"))
    res = s.send_command('Z\r\n')
    print(res)
    ser = serial.Serial(
        port="/dev/ttyUSB0",
        baudrate=19200,
        timeout=1)
    while True:
        ans = (ser.readline()).decode('utf-8')
        print(ans)
        # sleep()

def test_info():
    s = SBAWrapper()
    #pump_off = s.send_command('P0\r\n')
    #print(pump_off)
    res = s.send_command('?\r\n')
    print(res)
    ser = serial.Serial(
        port="/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DN03WQZS-if00-port0",
        baudrate=19200,
        timeout=1)
    while True:
        ans = (ser.readline()).decode('utf-8')
        print(ans)
        # sleep()

if __name__=="__main__":
    read_loop()
    #test_calibration()
    #test_info()