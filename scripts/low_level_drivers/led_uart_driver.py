#!/usr/bin/env python
# -*- coding: utf-8 -*-

#from typing import Tuple, Optional
import serial
import logging
import six
from time import sleep
import sys

# there must be uart wrapper object that realizes all
# methods for communication with our GIC

# Table driver crc16 algorithm.  The table is well-documented and was
# generated in this case by using pycrc (https://github.com/tpircher/pycrc)
# using the following command-line:
#
# ./pycrc.py --model=ccitt --generate table

logger = logging.getLogger('Worker.Units.Led_wrapper.LedUartWrapper')
handler = logging.StreamHandler(sys.stdout)
handler.setLevel(logging.DEBUG)
logger.addHandler(handler)

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


class UartWrapper:
    """
    taht class realizes all commands from IGC (Impulse Current Generator)
    documentation as its own methods
    must generate CRC and should be able to parse all commands
    it works as c-style structure - collects everything about uart
    in a big bunch
    # Кадр [COMMAND] может быть представлен следующим образом:
    # command structure PREAMBLE DIRECTION LENGTH TYPE PAYLOAD CRC MSB CRC LSB
    # ------------------  \x55     \xCC      \x01 \x14           \x8B    \x7C
    #
    # Кадр [RESPONSE] может быть представлен следующим образом:
    #     PREAMBLE DIRECTION LENGTH TYPE PAYLOAD CRC MSB CRC LSB
    #        0x55     0xAA    0x02  0x02   0x07   0x34     0x12
    #     SET_C b'\x55\xCC\x04\x0B\x01\x00\x00\x25\xB1')
    """
    def __init__(self,
                 devname = '/dev/ttyUSB0',
                 baudrate = 19200,
                 timeout = 10
                 ):
        self.dev = devname
        self.baud = baudrate
        self.timeout = timeout

    def send_command(self,
                     com,
                     log_comment= None
                     ):
        ans = None
        logger.debug("-------------------------------")
        if(log_comment):
            logger.debug("Sending {}".format(log_comment))
        else:
            logger.debug("We want to send this:")
        logger.debug(self.parse_command(com))
        try:
            ser = serial.Serial(port=self.dev, baudrate=self.baud, timeout=self.timeout)
            ser.write(com)
        except Exception as e:
            logger.debug("Error happened while write: {}".format(e))
            logger.debug("-------------------------------")
            return ans

        try:
            ans = ser.read(len(com))
        except Exception as e:
            logger.debug("Error happened while read: {}".format(e))
            logger.debug("-------------------------------")
            return ans

        if(not ans or (len(ans) != len(com))):
            logger.debug("Broken answer from GIC: {}".format(ans))
            logger.debug("-------------------------------")
        else:
            logger.debug("Succesfully got answer from GIC:")
            logger.debug(self.parse_command(ans))
        return ans

    def parse_command(self, com):
        # parse content of command
        data_length = com[2]
        length = len(com)
        parsed_output = ""
        logger.debug("-------------------------------")
        logger.debug("Parsed command {} ".format(com.hex()))
        logger.debug("{} - header byte ".format(hex(com[0])))
        logger.debug("{} - destination byte".format(hex(com[1])))
        logger.debug("{} - length of command".format(hex(com[2])))
        logger.debug("{} - type of command".format(hex(com[3])))
        if data_length > 1:
            # parse content of command
            for i in range(4, 4 + data_length - 1):
                logger.debug("{} - data byte".format(hex(com[i])))
        else:
            pass
        logger.debug("{} - last byte of CRC16 ccitt control sum".format(hex(com[length - 2])))
        logger.debug("{} - first byte of CRC16 ccitt control sum".format(hex(com[length - 1])))
        logger.debug("-------------------------------")
        return parsed_output

    def create_command(self,
                       preamble = b'\x55',
                       direction = b'\xCC',
                       length = None,
                       ctype = b'\x01',
                       data = None
                       ):
        command = bytearray()
        # print("preamble ", preamble)
        command.extend(preamble)
        # print("direction ", direction)
        command.extend(direction)
        if(not length):
            # length of command is length of data + 1 byte of command_type
            if(data):
                lenn = len(bytearray(data)) + 1
                length = int.to_bytes(lenn, 1, byteorder = 'big')
                command.extend(length)
            else:
                length = b'\x01'
                command.extend(length)
        else:
            command.extend(length)
        # print("length ", length)
        # print("ctype ", ctype)
        command.extend(ctype)
        if(data):
            command.extend(data)
        # print("data ", data)
        # crc should be calculated only for LENGTH | TYPE | DATA fields
        payload = bytearray()
        payload.extend(length)
        payload.extend(ctype)
        if(data):
            payload.extend(data)
        crc_raw = self.crc16_ccitt(payload)
        # print("crc_raw ", hex(crc_raw))
        crc_bytes = crc_raw.to_bytes(2, byteorder='little')  # byteorder='little'
        # its important
        # print("crc_bytes ", crc_bytes)
        command.extend(crc_bytes)
        return command

    def create_new_profile(self):
        pass

    # def update_params(self):
    #     pass
    # for future if need be

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
        for byte in six.iterbytes(data_):
            crc = (((crc << 8) & 0xff00) ^ tab[((crc >> 8) & 0xff) ^ byte])
        return crc & 0xffff

    def simple_command(self,
                       ACK= b'\x00',
                       NACK= b'\x80',
                       ctype = b'\x00',
                       data = None,
                       name = None
                       ):
        # there is a simple command template
        command = self.create_command(ctype=ctype, data=data)
        ans = self.send_command(command, log_comment=name)
        if ans:
            answer = bytearray(ans)
            if ACK in answer:
                logger.debug("There is ACK flag 0x{} in answer ".format(ACK.hex()))
                logger.debug("-------------------------------")
            if NACK in answer:
                logger.debug("There is NACK flag 0x{} in answer ".format(NACK.hex()))
                logger.debug("Something went wrong in GIC")
                logger.debug("-------------------------------")
            return ans
        else:
            logger.debug("Something went wrong, we got no answer")
            logger.debug("-------------------------------")
            return ans

    def PING_PONG(self):
        PING_PONG = bytearray(b'\x55\xCC\x02\x00\x00\xFC\xA2')
        pass

    def GET_STATUS(self):
        # GET_STATUS = bytearray(b'\x55\xCC\x01\x01\x1F\x3E')
        # there we must parse data in answer
        # but maybe later
        return self.simple_command(
                        ACK= b'\x01',
                        NACK=b'\x81',
                        ctype=b'\x01',
                        data=None,
                        name="GET_STATUS"
                       )

    def START(self):
        # START = bytearray(b'\x55\xCC\x01\x02\x7C\x0E')
        return self.simple_command(
                        ACK= b'\x02',
                        NACK=b'\x82',
                        ctype=b'\x02',
                        data=None,
                        name="START"
                       )

    def STOP(self):
        # STOP = bytearray(b'\x55\xCC\x01\x03\x5D\x1E')
        return self.simple_command(
                        ACK= b'\x03',
                        NACK=b'\x83',
                        ctype=b'\x03',
                        data=None,
                        name="STOP"
                       )

    def GET_PROFILE(self, num = b'\x01'):
        # GET_PROFILE = bytearray(b'\x55\xCC\x02\x04\x04\xBC\x2E')
        # -> DATA – uint8_t[0...4]
        # <- DATA – uProfile[1] – вернет требуемый профайл
        return self.simple_command(
                        ACK= b'\x04',
                        NACK=b'\x84',
                        ctype=b'\x04',
                        data=num,
                        name="GET_PROFILE"
                       )

    def START_CONFIGURE(self):
        # START_CONFIGURE = bytearray(b'\x55\xCC\x01\x05\x9B\x7E')
        return self.simple_command(
                        ACK= b'\x05',
                        NACK=b'\x85',
                        ctype=b'\x05',
                        data=None,
                        name="START_CONFIGURE"
                       )

    def EXIT_WITHOUT_SAVING(self):
        # EXIT_WITHOUT_SAVING = bytearray(b'"\x55\xCC\x01\x06\xF8\x4E')
        return self.simple_command(
                        ACK= b'\x06',
                        NACK=b'\x86',
                        ctype=b'\x06',
                        data=None,
                        name="EXIT_WITHOUT_SAVING"
                       )


    def FINISH_CONFIGURE_WITH_SAVING(self):
        # FINISH_CONFIGURE_WITH_SAVING = bytearray(b'\x55\xCC\x01\x07\xD9\x5E')
        return self.simple_command(
                        ACK= b'\x07',
                        NACK=b'\x87',
                        ctype=b'\x07',
                        data=None,
                        name="FINISH_CONFIGURE_WITH_SAVING"
                       )

    def SET_PROFILE(self):
        # WRONG CRC !!
        SET_PROFILE = bytearray(b'\x55\xCC\x02\x08\x00\x00\x00')

    def LOAD_PROFILE_TO_EEPROM(self):
        LOAD_PROFILE_TO_EEPROM = bytearray(
            b'\x55\xCC\x12\x09\x00\xC8\x00\xC8\x00\xFA\x00\xFA\x00\x05\x00\x05\x00\x03\x00\x00\x01\x00\x00')

    def SAVE_PROFILE_TO_EEPROM(self):
        SAVE_PROFILE_TO_EEPROM = bytearray(b'\x55\xCC\x02\x0A\x00\x00\x00')

    def SET_CURRENT(self, channel= 0, value= 200):
        # SET_CURRENT = bytearray(b'\x55\xCC\x04\x0B\x01\xE8\x03\x00\x00')
        # SET_CURRENT_200_1 = bytearray(b'\x55\xCC\x04\x0B\x01\xC8\x00\xD8\x2E')
        # SET_CURRENT_200_0 = bytearray(b'\x55\xCC\x04\x0B\x00\xC8\x00\xE8\x19')
        # SET_CURRENT_50_1 = bytearray(b'\x55\xCC\x04\x0B\x01\x32\x00\xD2\xD2')
        data = bytearray()
        data.extend(int.to_bytes(channel, 1, byteorder='big'))
        data.extend(int.to_bytes(value, 2, byteorder='little'))
        return self.simple_command(
                        ACK= b'\x0B',
                        NACK=b'\x8B',
                        ctype=b'\x0B',
                        data=data,
                        name="SET_CURRENT_{}_{}".format(channel, value)
                       )

    def GET_CURRENT(self):
        GET_CURRENT = bytearray(b'\x55\xCC\x02\x0C\x00\x00\x00')

    def SET_PULSE_LENGTH(self):
        SET_PULSE_LENGTH = bytearray(b'\x55\xCC\x04\x0D\x00\xFF\x03\x00\x00')

    def GET_PULSE_LENGTH(self):
        GET_PULSE_LENGTH = bytearray(b'\x55\xCC\x02\x0E\x00\x00\x00')

    def SET_PULSE_INTERVAL(self):
        SET_PULSE_INTERVAL = bytearray(b'\x55\xCC\x03\x0F\x7F\x00\x00\x00')

    def GET_PULSE_INTERVAL(self):
        GET_PULSE_INTERVAL = bytearray(b'\x55\xCC\x01\x10\x00\x00')

    def SET_LIMITATION(self):
        SET_LIMITATION = bytearray(b'\x55\xCC\x04\x11\x00\xE8\x00\x00\x00')

    def GET_LIMITATION(self):
        GET_LIMITATION = bytearray(b'\x55\xCC\x02\x12\x00\x00\x00')

    def SET_SETTINGS(self):
        SET_SETTINGS = bytearray(b'\x55\xCC\x02\x13\x00\xDC\xF4')

    def GET_SETTINGS(self):
        GET_SETTINGS = bytearray(b'\x55\xCC\x01\x14\x8B\x7C')

    def DEBUG(self):
        DEBUG = bytearray(b'\x55\xCC\x07\x00\x44\x45\x42\x55\x47\x00\x39\x0A')

    def RESET(self):
        RESET = bytearray(b'\x55\xCC\x07\x15\x52\x45\x53\x45\x54\x00\x00\x00')

if __name__ == "__main__":
    a = UartWrapper()
    # start = (a.create_command(
    #     length=None,
    #     ctype=b'\x02',
    #     data=None
    # ))
    #
    # print(a.parse_command(start))
    # print(a.send_command(start)[0])
    # print(a.send_command(start)[1])
    # print(a.START())
    # print(a.GET_STATUS()[1])
    print(a.START_CONFIGURE()[1])
    print(a.SET_CURRENT(0, 10)[1])
    print(a.SET_CURRENT(1, 10)[1])
    print(a.FINISH_CONFIGURE_WITH_SAVING()[1])
    print(a.START()[1])
    sleep(20)
    print(a.STOP()[1])