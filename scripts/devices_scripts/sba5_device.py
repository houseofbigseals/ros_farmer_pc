#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from std_msgs.msg import String
import serial
from data_scripts.custom_logger import CustomLogger
from ros_farmer_pc.srv import SBA5Device
import re

#
# To initiate a command, the USB port or RS232 port sends an ASCII character or string.
# A single character command is acted on immediately when the character is received.
# A string command is acted on after the command string terminator <CR> is received.
# The command can be sent with or without a checksum. If a checksum is sent, a “C” follows
# the checksum value.
# For example,
# Device sends command without checksum: S,11,1<CR>
# Device sends command with checksum: S,11,1,043C<CR>
# On successfully receiving a command string, the SBA5+ sends an acknowledgement by
# echoing back to all the ports the Command String and “OK”, each terminated with a <CR>
# and<linefeed>.

# Axxx<CR> Time [minutes] between zero operations
# Bxxx<CR> Averaging limit for CO 2 running average.
# Cxxx<CR> Number of digits to the right of the decimal point for ccc.ccc. range: 0-3
# (integer)
# Dxxx<CR> Determines if there is a zero operation at warmup
# Exxx<CR> Zero operation duration, argument is S, M, or L
# Fxxx<CR> Measurement string format. 0 to 255 to enable individual outputs
# H<min>,<max>CR Analog output H2OVOUT hardware scaling
# Jxxx<CR> Sets the mode of the spare I/O pin. See Spare I/O Line on page 24.
# Kxxx<CR> Turns source lamp on or off
# Lxxx<CR> Low CO 2 In [ppm] alarm.
# M Display a measurement.
# N Returns external board voltage and Spare I/O voltage
# O<min>,<max>CR Analog output CO2VOUT hardware scaling
# Pxxx<CR> Turns on-board pump on or off (if installed)
# S,11,xxx<CR> Sets the measurement string output interval in seconds. 0.1-36000.
# S,9,xxx<CR> Sets pump voltage from 0-100%
# S,16, xxx<CR> CRC appended to each output string. 1 enables, 0 disables
# Txxx<CR> Sets IRGA thermostats temperature
# Uxxx<CR> Sets the user scale factor (for user calibrations)
# V Returns the SBA-5 serial numbers and firmware version numbers
# Wxxx<CR> Defines the measurement and zero ports on the solenoid valve.
# Z Perform a zero operation.
# ! Turns measurement display off.
# @ Turns measurement display on.
# ? Display the SBA-5 configuration currently in use.
# ] Restore the factory default configuration.

# Measurement Commands
# M Display a measurement
# ! Turns measurement display off.
# @ Turns measurement display on.
# S,11,xxx<CR> Sets the measurement string output interval in seconds. 0.1-36000. Default is
# 1.0 sec. (Older SBA-5s had a fixed output interval of 1.6 sec)
# S,16, xxx<CR> CRC appended to each output string. 1 enables, 0 disables. Default disabled.

# Measurement Command Response
# Measurement format: M aaaaa bbbbb ccc.ccc dd.d ee.eeee ff.ffff gggg hh.h ii.i j
# aaaaa Zero A/D [counts], from last autozero sequence
# bbbbb Current A/D [counts]
# ccc.cc Measured CO2 [ppm],
# dd.d Average IRGA temperature [°C],
# ee.e Humidity [mbar], if humidity sensor is installed
# ff.f Humidity sensor temperature [°C], if humidity sensor is installed,
# gggg Atmospheric pressure in IRGA [mbar],
# hh.h IRGA detector temperature [°C],
# ii.i IRGA source temperature [°C],
# j Status/Error code. Continuously displayed measurements do not display the j but
# instead display a text message .
#
# 0 - No errors
# 1 - aaaaa less than 25000 counts
# 2 - dd.d less than 5 °C from user specified temperature
# 3 - dd.d greater than 5 °C from user specified temperature
# 4 - ccc.ccc less than range from L command
# 5 - ee.eeee greater than 90 mbar
# 6 - Board voltage less than 4V


# Measurement String Format Command:
# Fxxx<CR>
# Enables or disables individual measurement fields in the output measurement
# string. Range: 0-255 (integer). For each field desired in the output string, sum
# values from following list:
# aaaaa and bbbbb enabled with value =128,
# dd.d enabled with value =64,
# ee.eeee and ff.ffff enabled with value=32,
# gggg enabled with value=16,
# hh.h and ii.i enabled with value=08,
# j enabled with bit value=04.
# ccc.ccc is always present in output string.
# For example, when value is 212 (=128+64+16+4) the output string will be “M
# aaaaa bbbbb ccc.ccc dd.d gggg j”


# Zero Valve Related Commands
# Z Perform a zero operation.
# Dxxx<CR> Determines whether a zero operation is performed on completion of initial
# warmup or not. If not, then CO2 readings are computed with a previously
# stored zero reading that may produce inaccurate results. Recommended
# practice (and the default) is to perform a zero on power-up.
# Sending “D1” enables the power-up zero, and the string “Zpup=1” is shown in
# the configuration status (in response to a ? command). Sending “D0” disables
# the power-up zero, and the string “Zpup=0” is shown.
# E<char>CR
# Zero operation duration. char: “S” = 21 second,
# “M” = 40 seconds,
# “L” = 90 seconds.
# Default is Short in which the autozero sequence is approximately 20 sec long.
# Longer duration zero cycles can be useful if the flow rate through the SBA-5 is
# lower than 100 ml/min or the measured gas concentration is above 10,000
# ppm to insure fully purging measurement gas from the cell prior to recording a
# zero reading.
# Axxx<CR>
# Time [minutes] between zero operations. range: 0-10000 (integer, but can be
# negative). Recommended maximum setting is 20 minutes. Longer time
# between zero cycles can reduce instrument accuracy.
# Sending the A command with any non-zero value will cause an immediate zero
# operation, followed by subsequent zero operations every value minutes.
# Normally, the SBA-5 performs a series of zero operations at power-up while
# the temperature is stabilizing. The time between these initial zero operations
# is a geometric progression starting at 2 minutes, then 4 minutes, then 8
# minutes, etc. up to the maximum time between zeros as specified in the A
# command.
# It is possible, but not recommended, to disable these progressive zero
# operations during startup by setting the value in the A command to a negative
# number. For example, “A-10” will disable the progressive zeros if the
# configuration is saved with the X command, so that on the next power-up, the
# first timed zero occurs after 10 minutes (there still can be a power-up zero
# immediately after warm-up is complete, depending on the setting of the D
# command and Zpup).
# A0 disables all timed zeros and all progressive zeros. See D command to also
# disable the power-up zero. This is not a recommended setting.
# Wxxx<CR>
# Defines which port of the zero valve has the zero CO2 gas, and which has the
# sample gas to be measured. W0 (Zdir=0) is default and means the zero gas is
# plumbed to the white plastic port on the zero solenoid valve, and the
# measurement port is the metal port. W1 (Zdir=1) means the zero gas is
# plumbed to the metal port, and the measurement gas is the plastic port. Valve
# definitions change immediately upon sending the command.
# Note: In firmware V2.03 and earlier, the “W” command was a single character
# toggle-type command that changed the port state depending on what the
# current state was. In V2.04, it was changed to require a single argument 0 or
# 1


# CO2 Related Commands
# Cxxx<CR> Number of digits to the right of the decimal point for ccc.cc. range: 0-2
# (integer).
# Uxxx<CR> User Scale Factor. range: 0.1-10.0 (floating point). Default is 1.000. Scale
# factor applied to all reported CO2 values to allow user calibrations. There is
# no calibration ‘routine’, The user must calculate their desired scale factor from
# measurements and compute the USF = desired CO 2 ppm / reported CO 2 ppm.
# For example, if the SBA-5 reported 1995 ppm, when calibration gas of known
# 2000 ppm was sampled, then USF = 2000/1995 = 1.0025, and the command
# “U1.0025” would make the SBA-5 read 2000 ppm. User should set USF to
# 1.00 prior to performing recalibration measurements.
#
# Lxxx<CR> Low CO 2 In [ppm] alarm. range: 0-100000 (floating point). In typical
# environmental applications, a CO 2 reading in measurement mode of less than
# 350 ppm indicates a problem with the autozero operation, such as the zero
# gas is not connected, the CO 2 absorber is exhausted, or the zero valve is not
# operating. The Low CO 2 Error helps identify those common problems before
# the abnormal readings can affect subsequent data. This value can be
# adjusted to suit a particular operating environment or can be eliminated
# completely by setting the value to 0.
# Bxxx<CR> Averaging limit for CO2 running average. 0 – no averaging.
# Normally, an exponential running average algorithm is implemented with a
# time response to a step change of 3.5 seconds to 66% of final value and 16.4
# seconds to 99% of final value. If a new instrument reading differs from the
# current running average by more than the Averaging Limit xxx, a new running
# average is begun. Thus when the CO 2 concentration is changing rapidly, the
# averaging is eliminated and the instrument can track changes at the basic
# instrument data rate of 1.0 seconds. When the Averaging Limit value is set to
# 0, no running average is performed. The default Averaging Limit value is 6
# ppm. The running averaging is applied to displayed data and analog output
# signals.
# O<min>,<max>CR
# Analog output CO2VOUT hardware scaling
# min is CO 2 ppm represented by 0 volts, range: 0-100000 (integer),
# max is CO 2 ppm represented by 5 volts, range: 0-100000 (integer).
# H<min>,<max>CR
# Analog output H2OVOUT hardware scaling
# min is H 2 O mbar represented by 0 volts, range: 0-40 (integer),
# max is H 2 O mbar represented by 5 volts, range: 0-40 (integer).
# Other Commands
# Jxxx<CR>
# “J0” sets the spare I/O to digital output representing data valid (default).
# “J1” sets the spare I/O to be a 0-1.2V analog input. In this mode the output of
# the “M” line is modified to include the analog input voltage in millivolts with no
# decimal digits just before the error status field.
# In firmware V2.02 and earlier, there was no J command, and spare I/O was
# used only for a data valid indication.
# Kxxx<CR>
# Turns source lamp on(1) or off(0). This is useful if the SBA-5 is kept powered
# 24/7, but the CO 2 is only read occasionally (i.e. once per hour). By turning the
# lamp off when not in use, the life of the source lamp is extended.
# N Returns two voltage measurements, external board voltage and Spare I/O
# voltage
# Pxxx<CR> Turns the onboard pump from on or off, if one is installed.
# Sending “P0” turns pump off. Sending “P1” turns pump on.
# S,9,xxx<CR> Set the pump power 0-100% when the optional on-board is installed. Default
# 50%.
# Txxx<CR> Sets the control temperature of the IRGA thermostats in degrees C. Range is
# 27 to 60. Default is 55. This is useful for operation in low ambient temperature
# environments to save power. However, CO 2 accuracy will be affected if the
# control temperature is changed from the factory setting.
# V Returns the SBA-5 serial numbers and firmware version numbers
# ? Display the SBA-5 configuration currently in use (the volatile memory working
# area).
# ] Restore the factory default configuration and calibration to the volatile memory
# working area.


# /dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DN03WQZS-if00-port0


class SBA5DeviceException(Exception):
    pass

# Command must ends with \r\n !
# Its important


#  use \r\n !


class SBA5DeviceServer(object):
    """
    class to receive messages from other ros nodes, handle it
    send messages to PP-systems SBA-5 IRGA and parse its output back as response
    """
    def __init__(self):
        # hardcoded constants
        self._success_response = "success: "
        self._error_response = "error: "


        # start node
        rospy.init_node('sba5_device_server')

        # get roslaunch params and reinit part of params
        self._logname = rospy.get_param('~sba5_log_name', 'sba5')
        self._log_node_name = rospy.get_param('~sba5_log_node_name', 'sba5_log_node')
        self._port = rospy.get_param('~sba5_port', '/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DN03WQZS-if00-port0')
        self._baudrate = rospy.get_param('~sba5_baudrate', 19200)
        self._timeout = rospy.get_param('~sba5_timeout', 0.2)
        self._service_name = rospy.get_param('~sba5_service_name', 'sba5_device')

        # create log topic publisher
        self._log_pub = rospy.Publisher(self._log_node_name, String, queue_size=10)

        # logger
        self._logger = CustomLogger(name=self._logname, logpub=self._log_pub)
        self._logger.debug("sba5_device_server init")

        # service
        self._service = rospy.Service(self._service_name, SBA5Device, self.handle_request)
        self._loop()


    def _loop(self):
        #self._logger.debug("led_device_server ready to serve")
        rospy.spin()

    def handle_request(self, req):

        self._logger.debug("+++++++++++++++++++++++++++++++++++++++++++++++")
        self._logger.debug("+++++++++++++++++++++++++++++++++++++++++++++++")
        self._logger.debug("we got request: {}".format(req))

        if not req.command:
            # if we got empty string
            resp = self._error_response + 'empty_command'
            return resp

        elif req.command == 'get_info':
            try:
                ans = self.get_info()
                resp = self._success_response + ans
                return resp
            except SBA5DeviceException as e:
                resp = self._error_response + e.args[0]
                return resp

        elif req.command == 'measure_full':
            try:
                ans = self.do_measurement()
                resp = self._success_response + ans
                return resp
            except SBA5DeviceException as e:
                resp = self._error_response + e.args[0]
                return resp

        elif req.command == 'measure_co2':
            try:
                ans = self.do_measurement()

                pattern = re.compile(r'M \d+ \d+ (\d+.\d+) \d+.\d+ \d+.\d+ \d+.\d+ \d+ \d+\r\n')
                res = pattern.findall(ans)

                self._logger.debug("we have found this {}".format(res[0]))
                resp = self._success_response + str(res[0])
                return resp

            except SBA5DeviceException as e:
                resp = self._error_response + e.args[0]
                return resp

        elif req.command == 'recalibrate':
            try:
                ans = self.do_calibration()
                resp = self._success_response + ans
                return resp
            except SBA5DeviceException as e:
                resp = self._error_response + e.args[0]
                return resp

        elif req.command == 'init':
            try:
                ans = self.set_initial_params()
                resp = self._success_response + ans
                return resp
            except SBA5DeviceException as e:
                resp = self._error_response + e.args[0]
                return resp

        else:
            resp = self._error_response + 'unknown command'
            return resp
        pass

    # =========================== low level commands ===================================

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
                port=self._port,
                baudrate=self._baudrate,
                timeout=self._timeout
            )
            bcom = command.encode('utf-8')
            ser.write(bcom)

        except Exception as e:
            raise SBA5DeviceException("SBAWrapper error while send command: {}".format(e))
            # self._logger.error("SBAWrapper error while send command: {}".format(e))
        # then try to read answer
        # it must be two messages, ended with \r\n
        try:
            ser = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                timeout=self._timeout
            )
            echo = (ser.readline()).decode('utf-8')
            status = (ser.readline()).decode('utf-8')
            return echo+status
            #return status

        except Exception as e:
            raise SBA5DeviceException("SBAWrapper error while read answer from command: {}".format(e))
            # self._logger.error("SBAWrapper error while read answer from command: {}".format(e))

    def set_initial_params(self):
        # TODO: mb we need protection from calling that method twice?
        # async part of init
        # we have to send some commands before start regular work of unit
        self._logger.debug("Second part of CO2Sensor init")
        # ans = await self.sensor.send_command("?\r\n")
        # self.logger.info("Command ?, answer: {}".format(ans))
        # if not "OK" in ans:
        #     self.logger.info("CO2SensorError: {}".format(ans))
        #     return "CO2SensorError: {}".format(ans)
        ans_ = ""
        # we need to shut down auto measurements
        ans = self.send_command("!\r\n")
        ans_ += ans
        self._logger.debug("Command !, answer: {}".format(ans)[:-1])
        # we need to shut down auto zero operations
        ans += self.send_command("A0\r\n")
        ans_ += ans
        self._logger.debug("Command A0, answer: {}".format(ans)[:-1])
        # we need to set format of output
        ans += self.send_command("F252\r\n")
        ans_ += ans
        self._logger.debug("Command F252, answer: {}".format(ans)[:-1])
        # we need to start pump
        ans += self.send_command("P1\r\n")
        ans_ += ans
        self._logger.debug("Command P1, answer: {}".format(ans)[:-1])
        # set medium time of calibration
        ans += self.send_command("EL\r\n")
        ans_ += ans
        self._logger.debug("Command EL, answer: {}".format(ans)[:-1])
        return ans_

    def get_info(self):
        # only first line of answer
        ans = self.send_command("?\r\n")
        self._logger.debug("Getting info from SBA5")
        return ans[:-1]

    def do_calibration(self):
        ans = self.send_command("Z\r\n")
        self._logger.debug("Starting calibration of SBA5")
        return ans

    def do_measurement(self):
        ans = self.send_command("M\r\n")
        self._logger.debug("Do measure SBA5")
        self._logger.debug(("SBA5 result is {}".format(ans))[:-1])  #its try to remove last \n from here
        return ans

    def do_command(self, com):
        ans = self.send_command(com)
        self._logger.info("send {} command to SBA5".format(com)[:-1])
        return ans


if __name__ == "__main__":
    SBA5DeviceServer()