#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from std_msgs.msg import String
import serial
from data_scripts.custom_logger import CustomLogger
from ros_farmer_pc.srv import LedDevice, LedDeviceResponse

from future import *


# that file realizes all commands from IGC (Impulse Current Generator)
# documentation, as its own methods
# can generate CRC and  able to parse all commands
# it just collects everything about uart
# in a big bunch
#


# custom errors

class LedDeviceException(Exception):
    pass


# Кадр [COMMAND] может быть представлен следующим образом:
# command structure PREAMBLE DIRECTION LENGTH TYPE PAYLOAD CRC MSB CRC LSB
# ------------------  \x55     \xCC      \x01 \x14           \x8B    \x7C
#
# Кадр [RESPONSE] может быть представлен следующим образом:
#     PREAMBLE DIRECTION LENGTH TYPE PAYLOAD CRC MSB CRC LSB
#        0x55     0xAA    0x02  0x02   0x07   0x34     0x12
#     SET_C b'\x55\xCC\x04\x0B\x01\x00\x00\x25\xB1')

# Table driver crc16 algorithm.  The table is well-documented and was
# generated in this case by using pycrc (https://github.com/tpircher/pycrc)
# using the following command-line:
#
# ./pycrc.py --model=ccitt --generate table

CRC16_CCITT_TAB = \
        [
            0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
            0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
            0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
            0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
            0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
            0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
            0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
            0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
            0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
            0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
            0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
            0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
            0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
            0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
            0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
            0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
            0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
            0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
            0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
            0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
            0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
            0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
            0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
            0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
            0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
            0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
            0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
            0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
            0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
            0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
            0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
            0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
        ]


class LedDeviceServer(object):
    """
    class to receive messages from other ros nodes, handle it
    send messages to led driver and parse its output back as response
    """
    def __init__(self):
        # hardcoded constants
        self._success_response = "success"
        self._error_response = "error: "


        # start node
        rospy.init_node('led_device_server')

        # get roslaunch params and reinit part of params
        self._logname = rospy.get_param('~led_log_name', 'LED')
        self._log_node_name = rospy.get_param('~led_log_node_name', 'led_log_node')
        self._port = rospy.get_param('~led_port', '/dev/ttyUSB0')
        self._baudrate = rospy.get_param('~led_baudrate', 19200)
        self._timeout = rospy.get_param('~led_timeout', 10)
        self._service_name = rospy.get_param('~led_service_name', 'led_device')
        self._max_red_current = rospy.get_param('~led_max_red_current', 250)
        self._min_red_current = rospy.get_param('~led_min_red_current', 10)
        self._max_white_current = rospy.get_param('~led_max_white_current', 250)
        self._min_white_current = rospy.get_param('~led_min_white_current', 10)

        # create log topic publisher
        self._log_pub = rospy.Publisher(self._log_node_name, String, queue_size=10)

        # logger
        self._logger = CustomLogger(name=self._logname, logpub=self._log_pub)
        self._logger.debug("led_device_server init")

        # service
        self._service = rospy.Service(self._service_name, LedDevice, self.handle_request)
        self._loop()


    def _loop(self):
        #self._logger.debug("led_device_server ready to serve")
        rospy.spin()

    def handle_request(self, req):
        # check params from request
        # at first find what user wants from us:
        self._logger.debug("+++++++++++++++++++++++++++++++++++++++++++++++")
        self._logger.debug("+++++++++++++++++++++++++++++++++++++++++++++++")
        self._logger.debug("we got request: {}".format(req))

        if not req.command:
            # if we got empty string
            resp = self._error_response + 'empty_command'
            return resp

        elif req.command == 'start':
            try:
                self._start()
                resp = self._success_response
                return resp
            except LedDeviceException as e:
                resp = self._error_response + e.args[0]
                return resp

        elif req.command == 'stop':
            try:
                self._stop()
                resp = self._success_response
                return resp
            except LedDeviceException as e:
                resp = self._error_response + e.args[0]
                return resp

        elif req.command == 'start_configure':
            try:
                self._start_configure()
                resp = self._success_response
                return resp
            except LedDeviceException as e:
                resp = self._error_response + e.args[0]
                return resp

        elif req.command == 'finish_configure_and_save':
            try:
                self._finish_configure_with_saving()
                resp = self._success_response
                return resp
            except LedDeviceException as e:
                resp = self._error_response + e.args[0]
                return resp

        elif req.command == 'set_current':
            if req.red < self._min_red_current or req.red > self._max_red_current:
                resp = self._error_response + 'too big red current'
                return resp
            elif req.white < self._min_white_current or req.white > self._max_white_current:
                resp = self._error_response + 'too big white current'
                return resp
            else:
                try:
                    # red
                    self._set_current(0, int(req.red))
                    # white
                    self._set_current(1, int(req.white))
                    resp = self._success_response
                    return resp

                except LedDeviceException as e:
                    resp = self._error_response + e.args[0]
                    return resp

        elif req.command == 'full_reconfigure':
            if req.red < self._min_red_current or req.red > self._max_red_current:
                resp = self._error_response + 'too big red current'
                return resp
            elif req.white < self._min_white_current or req.white > self._max_white_current:
                resp = self._error_response + 'too big white current'
                return resp
            else:
                try:
                    self._stop()
                    self._start_configure()
                    # red
                    self._set_current(0, int(req.red))
                    # white
                    self._set_current(1, int(req.white))
                    self._finish_configure_with_saving()
                    self._start()
                    resp = self._success_response
                    return resp

                except LedDeviceException as e:
                    resp = self._error_response + e.args[0]
                    return resp

        else:
            resp = self._error_response + 'unknown command'
            return resp

    # ==================== low level command wrappers ==================

    def send_command(self,
                     com,
                     log_comment=None
                     ):
        ans = None
        self._logger.debug("-------------------------------")
        if (log_comment):
            self._logger.debug("Sending {}".format(log_comment))

        else:
            self._logger.debug("We want to send this:")
        self._logger.debug(self.parse_command(com))
        try:
            ser = serial.Serial(port=self._port, baudrate=self._baudrate, timeout=self._timeout)
            ser.write(com)
        except Exception as e:
            self._logger.debug("Error happened while write: {}".format(e))
            self._logger.debug("-------------------------------")
            raise LedDeviceException("Error happened while write: {}".format(e))

        try:
            ans = ser.read(len(com))  # returns str in python2, oooooff
            self._logger.debug("We  have read {} bytes".format(len(ans)))

        except Exception as e:
            self._logger.debug("Error happened while read: {}".format(e))
            self._logger.debug("-------------------------------")
            raise LedDeviceException("Error happened while read: {}".format(e))

        if (not ans or (len(ans) != len(com))):
            self._logger.debug("Broken answer from GIC: {}".format(ans))
            self._logger.debug("-------------------------------")
            raise LedDeviceException("Broken answer from GIC: {}".format(ans))
        else:
            self._logger.debug("Succesfully got answer from GIC:")

            # lets try to decode to int
            byte_ans = bytearray()
            for b_ in ans:
                b_decoded = ord(b_)  # important when encode ser.read() output back to int
                self._logger.debug("Decoded answer byte: {}".format(hex(b_decoded)))
                byte_ans.extend([b_decoded])

            self._logger.debug(self.parse_command(byte_ans))
            return byte_ans

    def crc16_ccitt(self, data_, crc=0xffff):
        """Calculate the crc16 ccitt checksum of some data
        A starting crc value may be specified if desired.  The input data
        is expected to be a sequence of bytes (string) and the output
        is an integer in the range (0, 0xFFFF).  No packing is done to the
        resultant crc value.  To check the value a checksum, just pass in
        the data byes and checksum value.  If the data matches the checksum,
        then the resultant checksum from this function should be 0.
        """
        tab = CRC16_CCITT_TAB  # minor optimization (now in locals)
        # for byte in six.iterbytes(data_):
        for byte in data_:
            self._logger.debug("current byte is {}".format(hex(byte)))
            crc = (((crc << 8) & 0xff00) ^ tab[((crc >> 8) & 0xff) ^ byte])
            self._logger.debug("current crc is {}".format(hex(byte)))

        self._logger.debug("final crc is {}".format(hex(crc & 0xffff)))
        return crc & 0xffff

    def parse_command(self, com):
        # parse content of command
        data_length = com[2]
        length = len(com)
        parsed_output = ""
        self._logger.debug("-------------------------------")
        self._logger.debug("Parsed command ")
        for b_ in com:
            pass
            # self._logger.debug("{} - type of raw byte".format(type(b_)))
            # self._logger.debug("{} - type of raw byte, and hex value of it {} ".format(type(b_), hex(b_)))
            # self._logger.debug("{} - raw byte ".format(hex(b_)))
        self._logger.debug("------------------")
        self._logger.debug("{} - header byte ".format(hex(com[0])))
        self._logger.debug("{} - destination byte".format(hex(com[1])))
        self._logger.debug("{} - length of command".format(hex(com[2])))
        self._logger.debug("{} - type of command".format(hex(com[3])))
        if data_length > 1:
            # parse content of command
            for i in range(4, 4 + data_length - 1):
                self._logger.debug("{} - data byte".format(hex(com[i])))
        else:
            pass
        self._logger.debug("{} - last byte of CRC16 ccitt control sum".format(hex(com[length - 2])))
        self._logger.debug("{} - first byte of CRC16 ccitt control sum".format(hex(com[length - 1])))
        self._logger.debug("-------------------------------")
        return parsed_output

    def simple_command(self,
                       ACK=0x00,
                       NACK=0x80,
                       ctype=0x00,
                       data=None,
                       name=None
                       ):

        # data is list of ints or None

        # there is a simple command template
        command = self.create_command(ctype=ctype, data=data)
        ans = self.send_command(command, log_comment=name)
        if ans:
            answer = bytearray(ans)
            if ACK in answer:
                self._logger.debug("There is ACK flag {} in answer ".format(hex(ACK)))
                self._logger.debug("-------------------------------")
                return answer
            if NACK in answer:
                self._logger.debug("There is NACK flag {} in answer ".format(hex(NACK)))
                self._logger.debug("Something went wrong in GIC")
                self._logger.debug("-------------------------------")
                raise LedDeviceException("There is NACK flag {} in answer ".format(hex(NACK)))
        else:
            self._logger.debug("Something went wrong, we got no answer")
            self._logger.debug("-------------------------------")
            raise LedDeviceException("Something went wrong")

    def create_command(self,
                       preamble=0x55,
                       direction=0xCC,
                       length=None,
                       ctype=0x01,
                       data=None
                       ):
        command = bytearray()
        # print("preamble ", preamble)
        command.extend([preamble])
        # print("direction ", direction)
        command.extend([direction])
        if (not length):
            # length of command is length of data + 1 byte of command_type
            if (data):
                length = len(bytearray(data)) + 1
                # split and add to length

                # length = int.to_bytes(lenn, 1, byteorder = 'big')
                command.extend([length])
            else:
                length = 0x01
                command.extend([length])
        else:
            command.extend([length])
        # print("length ", length)
        # print("ctype ", ctype)
        command.extend([ctype])
        if (data):
            # data must be list or none
            command.extend(data)
            # print("data ", data)
        # crc should be calculated only for LENGTH | TYPE | DATA fields
        payload = bytearray()
        payload.extend([length])
        payload.extend([ctype])
        if (data):
            payload.extend(data)
        crc_raw = self.crc16_ccitt(payload)  # returns int
        self._logger.debug("{} - crc raw ".format(hex(crc_raw)))
        # crc_bytes = crc_raw.to_bytes(2, byteorder='little')  # byteorder='little'

        first_byte = ((crc_raw & 0xff00) >> 8)
        last_byte = (crc_raw & 0x00ff)

        self._logger.debug("{} - first crc byte".format(hex(first_byte)))

        self._logger.debug("{} - last crc byte".format(hex(last_byte)))
        # then reorder
        crc_bytes = bytearray([last_byte, first_byte])

        # its important
        # print("crc_bytes ", crc_bytes)
        command.extend(crc_bytes)
        return command

    # ======================= real commands ===========================

    def _start(self):
        # START = bytearray(b'\x55\xCC\x01\x02\x7C\x0E')
        return self.simple_command(
            ACK=0x02,
            NACK=0x82,
            ctype=0x02,
            data=None,
            name="START"
        )

    def _stop(self):
        # STOP = bytearray(b'\x55\xCC\x01\x03\x5D\x1E')
        return self.simple_command(
            ACK=0x03,
            NACK=0x83,
            ctype=0x03,
            data=None,
            name="STOP"
        )

    def _start_configure(self):
        # START_CONFIGURE = bytearray(b'\x55\xCC\x01\x05\x9B\x7E')
        return self.simple_command(
            ACK=0x05,
            NACK=0x85,
            ctype=0x05,
            data=None,
            name="START_CONFIGURE"
        )

    def _set_current(self, channel=0, value=100):
        # SET_CURRENT = bytearray(b'\x55\xCC\x04\x0B\x01\xE8\x03\x00\x00')
        # SET_CURRENT_200_1 = bytearray(b'\x55\xCC\x04\x0B\x01\xC8\x00\xD8\x2E')
        # SET_CURRENT_200_0 = bytearray(b'\x55\xCC\x04\x0B\x00\xC8\x00\xE8\x19')
        # SET_CURRENT_50_1 = bytearray(b'\x55\xCC\x04\x0B\x01\x32\x00\xD2\xD2')
        data = bytearray()
        data.extend([channel])
        # split value bytes and add in little-endian to data
        #
        #       first_byte = ((crc_raw & 0xff00) >> 8)
        #       last_byte = (crc_raw & 0x00ff)
        #       self._logger.debug("current byte is{}".format(hex(byte)))

        self._logger.debug("value raw {}".format(hex(value)))
        # crc_bytes = crc_raw.to_bytes(2, byteorder='little')  # byteorder='little'
        value_first_byte = ((value & 0xff00) >> 8)
        self._logger.debug("value first byte {}".format(hex(value_first_byte)))  # , value_first_byte)
        value_last_byte = (value & 0x00ff)
        self._logger.debug("value last byte {}".format(hex(value_last_byte)))  # , value_last_byte)
        # data.extend(int.to_bytes(channel, 1, byteorder='big'))
        # data.extend(int.to_bytes(value, 2, byteorder='little'))
        data.extend([value_last_byte, value_first_byte])
        self._logger.debug("final data array {}".format(data))
        return self.simple_command(
            ACK=0x0B,
            NACK=0x8B,
            ctype=0x0B,
            data=data,
            name="SET_CURRENT_{}_{}".format(channel, value)
        )

    def _finish_configure_with_saving(self):
        # FINISH_CONFIGURE_WITH_SAVING = bytearray(b'\x55\xCC\x01\x07\xD9\x5E')
        return self.simple_command(
            ACK=0x07,
            NACK=0x87,
            ctype=0x07,
            data=None,
            name="FINISH_CONFIGURE_WITH_SAVING"
        )

    def _exit_without_saving(self):
        # EXIT_WITHOUT_SAVING = bytearray(b'"\x55\xCC\x01\x06\xF8\x4E')
        return self.simple_command(
            ACK=0x06,
            NACK=0x86,
            ctype=0x06,
            data=None,
            name="EXIT_WITHOUT_SAVING"
        )


if __name__ == "__main__":
    a = LedDeviceServer()


