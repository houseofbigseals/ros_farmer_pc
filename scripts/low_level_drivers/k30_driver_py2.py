#!/usr/bin/env python
# -*- coding: utf-8 -*-
import serial
import time


# crc calculation using libscrc
# hex(libscrc.modbus(b'\xFE\x06\x00\x01\x7C\x07'))
# then split and reverse that bytes

class K30(object):
    """
    Simple wrapper for k30 co2 sensor
    https://www.co2meter.com/products/k-30-co2-sensor-module
    """
    def __init__(self,
                 devname="/dev/ttyUSB0",
                 baudrate=9600,
                 timeout=1
                 ):
        self.devname = devname
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = serial.Serial(
            port=self.devname,
            baudrate=self.baudrate,
            timeout=self.timeout
        )
        self.ser.flushInput()

    def get_co2(self):
        """
        Returns co2 in ppm and str description of operation
        :return:
        """
        read_co2_modbus = b"\x68\x04\x00\x03\x00\x01\xC8\xF3"
        res = "Get data from K-30 via UART\n"
        self.ser.flushInput()
        time.sleep(0.3)
        self.ser.write(read_co2_modbus)
        res += "Sent: {} \n".format(read_co2_modbus)
        time.sleep(0.3)
        resp = self.ser.read(7)
        resp_hex_str = "0x" + "".join([a.encode('hex') for a in resp])
        res += "This is resp : {} \n".format(resp_hex_str)
        high = ord(resp[3])
        low = ord(resp[4])
        co2 = (high * 256) + low
        res += "CO2 = {}".format(co2)
        # print ("This is resp : {}".format(resp_hex_str))
        # print(" CO2 = " + str(co2))
        return co2, res

    def get_info(self):
        """

        """
        read_status_modbus = b'\xFE\x04\x00\x00\x00\x01\x25\xC5'
        # print("Get info data from K-30 via UART\n")
        self.ser.flushInput()
        time.sleep(1)
        self.ser.write(read_status_modbus)
        time.sleep(1)
        resp = self.ser.read(7)
        resp_hex_str = "0x" + "".join([a.encode('hex') for a in resp])
        # print ("This is resp : {}".format(resp_hex_str))
        return resp

    def send_command(self, com, output_len):
        # print("Get info data from K-30 via UART\n")
        self.ser.flushInput()
        time.sleep(1)
        self.ser.write(com)
        time.sleep(1)
        resp = self.ser.read(output_len)
        resp_hex_str = "0x" + "".join([a.encode('hex') for a in resp])
        # print ("This is resp : {}".format(resp_hex_str))
        return resp

    def recalibrate(self, ctime=5):
        self.ser.flushInput()
        time.sleep(1)
        # clear status reg
        self.ser.write(b'\xFE\x06\x00\x00\x00\x00\x9D\xC5')
        time.sleep(1)
        resp = self.ser.read(8)
        resp_hex_str = "0x" + "".join([a.encode('hex') for a in resp])
        print(resp_hex_str)
        # send command to calibrate
        self.ser.write(b'\xFE\x06\x00\x01\x7C\x07\xAD\x07')
        time.sleep(1)
        resp = self.ser.read(8)
        resp_hex_str = "0x" + "".join([a.encode('hex') for a in resp])
        print(resp_hex_str)
        # wait 5 secs
        time.sleep(ctime)
        # check calibration status
        self.ser.write(b'\xFE\x03\x00\x00\x00\x01\x90\x05')
        time.sleep(1)
        resp = self.ser.read(7)
        resp_hex_str = "0x" + "".join([a.encode('hex') for a in resp])
        print(resp_hex_str)



def old_main():

    ser = serial.Serial("/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0003-if00-port0",
                        baudrate=9600, timeout=1)  # serial port may vary from pi to pi
    print("Get data from K-30 via UART\n")
    ser.flushInput()
    time.sleep(1)
    # cutoff_ppm = 1000  # readings above this will cause the led to turn red

    # read_co2 = (b"\xFE\x44\x00\x08\x02\x9F\x25")
    # read_temp = (b"\xFE\x44\x00\x12\x02\x94\x45")  # dont work really
    # read_rh = (b"\xFE\x44\x00\x14\x02\x97\xE5")
    # cmd_init = (b"\xFE\x41\x00\x60\x01\x35\xE8\x53")

    read_co2_modbus = (b"\x68\x04\x00\x03\x00\x01\xC8\xF3")

    for i in range(1, 10):
        ser.flushInput()

        ser.write(read_co2_modbus)
        time.sleep(1)
        resp = ser.read(7)
        resp_hex_str = "0x" + "".join([a.encode('hex') for a in resp])
        # resp_hex_str = hex(ord(resp))
        # print (type(resp)) #used for testing and debugging
        # print (len(resp))
        # print (resp)
        print ("This is resp : {}".format(resp_hex_str))
        high = ord(resp[3])
        low = ord(resp[4])
        # print(high)
        # print(low)
        co2 = (high * 256) + low
        print("i = ", i, " CO2 = " + str(co2))


if __name__ == "__main__":
    # old_main()
    sensor = K30(devname="/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0003-if00-port0",
                        baudrate=9600, timeout=1)
    print(sensor.get_info())
    while True:
        print(sensor.get_co2())
    # sensor.recalibrate()


