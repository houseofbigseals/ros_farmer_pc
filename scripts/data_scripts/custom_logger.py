#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import datetime


class CustomLogger(object):
    """
    Custom debug message wrapper
    It must be called only after init_node
    """
    def __init__(
            self,
            name,
            logpub=None,
            logdebug="debug",
            loginfo="info",
            logwarning="warning",
            logerror="error",
            # logoperator="OPERATOR",
            logsep=";"
    ):
        self._name = name
        self._logdebug = logdebug
        self._logerror = logerror
        self._loginfo = loginfo
        self._logwarning = logwarning
        self._logsep = logsep
        self._logpub = logpub
        # self._logoperator = logoperator

    def debug(self, _message):
        # _time=rospy.get_time()
        _time = datetime.datetime.now()
        msg = str(_time) + self._logsep + str(self._name) + self._logsep + self._logdebug + self._logsep + str(_message)
        # self._logpub.publish(msg)
        rospy.logdebug(msg)
        return msg

    def info(self, _message):
        # _time = rospy.get_time()
        _time = datetime.datetime.now()
        msg = str(_time) + self._logsep + str(self._name) + self._logsep + self._loginfo + self._logsep + str(_message)
        # self._logpub.publish(msg)
        rospy.loginfo(msg)
        return msg

    def warning(self, _message):
        # _time = rospy.get_time()
        _time = datetime.datetime.now()
        msg = str(_time) + self._logsep + str(self._name) + self._logsep + self._logwarning + self._logsep + str(_message)
        # self._logpub.publish(msg)
        rospy.logwarn(msg)
        return msg

    def error(self, _message):
        _time = datetime.datetime.now()
        msg = str(_time) + self._logsep + str(self._name) + self._logsep + self._logerror + self._logsep + str(_message)
        # self._logpub.publish(msg)
        rospy.logerr(msg)
        return msg

    # def operator(self, _message):
    #     _time = datetime.datetime.now()
    #     msg = str(_time) + self._logsep + str(self._name) + self._logsep + self._logoperator + self._logsep + str(_message)
    #     # self._logpub.publish(msg)
    #     rospy.logwarn(msg)
    #     return msg
