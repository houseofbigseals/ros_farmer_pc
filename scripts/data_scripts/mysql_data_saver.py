#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import datetime
from std_msgs.msg import String, Header
from rosgraph_msgs.msg import Log
from custom_logger import CustomLogger
from sensor_msgs.msg import Temperature
import time
import numpy as np
import pymysql

class MYSQLDataSaver(object):

    def generate_experiment_id(self, date_=None, type_=None, number_=None):
        """
        date - str
        type - str
        number - str
        """
        if not date_:
            date_ = datetime.datetime.now().strftime('%Y_%m_%d')
        if not type_:
            type_ = "TEST"
        if not number_:
            number_ = "0000"
        # experiment id is string like that YYYY_MM_DD_TYPE_NUMB
        # where YYYY_MM_DD is data of start
        # TYPE is short type of experiment (like TEST, TABLE, SEARCH etc)
        # NUMB is uniq number of experiment

        id_str = date_ + type_ + number_

        return id_str


    def __init__(self):
        # hardcoded constants
        self._success_response = "success: "
        self._error_response = "error: "

        # start node
        rospy.init_node('mysql_data_saver_server', log_level=rospy.INFO)

        # get roslaunch params

        # names for self topics
        self._logname = rospy.get_param('~mysql_data_saver_log_name', 'mysql_data_saver')
        self._log_node_name = rospy.get_param('~mysql_data_saver_log_node_name', 'mysql_data_saver_log')
        # self._service_name = rospy.get_param('~mysql_data_saver_service_name', 'mysql_data_saver')

        self._system_log_sub = rospy.Subscriber(
            name='/rosout_agg', data_class=Log,
            callback=self._log_callback,
            queue_size=50)
        # create log topic publisher
        self._log_pub = rospy.Publisher(self._log_node_name, String, queue_size=10)

        # start logger
        self._logger = CustomLogger(name=self._logname, logpub=self._log_pub)
        self._logger.debug("mysql_data_saver_server init")

        default_id = self.generate_experiment_id()
        # self._experiment_id = rospy.get_param('~mysql_data_saver_experiment_id', default_id)
        # experiment id is string like that YYYY_MM_DD_TYPE_NUMB
        # where YYYY_MM_DD is data of start
        # TYPE is short type of experiment (like TEST, TABLE, SEARCH etc)
        # NUMB is uniq number of experiment

        # TODO get params of sql database

        # TODO really get

        self.con = pymysql.connect(host='localhost',
                              user='admin',
                              password='admin',
                              # db='experiment',
                              charset='utf8mb4',
                              cursorclass=pymysql.cursors.DictCursor)



        self._raw_topics = rospy.get_param('~mysql_data_saver_raw_topics')
        # self._exp_topics = rospy.get_param('~mysql_data_saver_exp_topics')

        self._description = rospy.get_param('mysql_data_saver_experiment_description')


        # list to keep subscribers (for what?)
        self._subscribers_list = list()

        # check if tables in db exists, or update it
        self._create_database()

        # TODO insert into sensors table all sensors from .launch if not exists

        #  TODO and insert into experiment table current experiment if not exists

        # sub to all topics from parsed string
        self._logger.debug("creating raw subs")
        for topic in self._raw_topics:
            # for now we think that all raw_data_topics have type "sensor_msgs/Temperature"
            # it must be in architecture of all system !
            # so here we think that it is "sensor_msgs/Temperature" as default
            s = rospy.Subscriber(name=topic['name'], data_class=Temperature,
                                 callback=self._raw_data_callback, callback_args=topic,
                                 queue_size=2)
            print(s)
            self._subscribers_list.append(s)

        self._logger.debug("end of init, go to loop")

        self._loop()

    def _loop(self):
        rospy.spin()

    def _log_callback(self, log_msg):


        exp_id_ = self._description["experiment_number"]

        time_ = datetime.datetime.fromtimestamp(
            log_msg.header.stamp.to_sec()).strftime('%Y_%m_%d %H:%M:%S')

        level_ = log_msg.level
        # # Pseudo-constants
        # DEBUG = 1
        # INFO = 2
        # WARN = 4
        # ERROR = 8
        # FATAL = 16

        node_ = log_msg.name

        msg_ = log_msg.msg

        con = pymysql.connect(host='localhost',
                              user='admin',
                              password='admin',
                              # db='experiment',
                              charset='utf8mb4',
                              cursorclass=pymysql.cursors.DictCursor)

        cur = con.cursor()

        cur.execute("use experiment")

        comm_str = 'insert into logs' \
                   '(exp_id, time, level, node, msg)' \
                   'values("{}", "{}","{}", "{}", "{}")'.format(
            exp_id_, time_, level_, node_, msg_)

        print("comm_str: {}".format(comm_str))  # TODO: remove after debug
        try:
            cur.execute(comm_str)
        except Exception as e:
            print("Error while saving logs:")
            print(e)
            print(log_msg.msg)

        cur.execute('commit')
        con.close()

    def _raw_data_callback(self, data_message, topic_info):

        # topic_info must be dict and contain 'name', 'id', 'type', 'dtype', 'units', 'status':
        # example:
        # { name: '/bmp180_1_temp_pub', id: '2', type: 'temperature', dtype: 'float64', units: 'C', status:'raw },

        # example how to convert rospy time to datetime
        # (datetime.datetime.fromtimestamp(rospy.Time.now().to_sec())).strftime('%Y_%m_%d,%H:%M:%S')






        data_ = data_message.temperature
        self._logger.debug("data: {}".format(data_))

        time_ = datetime.datetime.fromtimestamp(
            data_message.header.stamp.to_sec()).strftime('%Y_%m_%d %H:%M:%S')
        self._logger.debug("time: {}".format(time_))

        sensor_ = topic_info["id"]
        self._logger.debug("sensor: {}".format(sensor_))

        exp_id_ = self._description["experiment_number"]
        self._logger.debug("exp_id: {}".format(exp_id_))

        con = pymysql.connect(host='localhost',
                              user='admin',
                              password='admin',
                              # db='experiment',
                              charset='utf8mb4',
                              cursorclass=pymysql.cursors.DictCursor)

        cur = con.cursor()

        cur.execute("use experiment")

        comm_str = 'insert into raw_data'\
                   '(exp_id, time, sensor_id, data)'\
                   'values("{}", "{}","{}", "{}" )'.format(
            exp_id_, time_, sensor_, data_)

        self._logger.debug("comm_str: {}".format(comm_str))

        cur.execute(comm_str)

        cur.execute('commit')
        con.close()


    def _create_database(self):
        self._logger.debug("create database")
        cur = self.con.cursor()

        cur.execute("CREATE DATABASE IF NOT EXISTS experiment")
        cur.execute("use experiment")

        # TODO load params from .launch and put to the database
        self._logger.debug("create table logs")

        cur.execute('create table if not exists logs'
                    ' ( log_id bigint unsigned primary key not null auto_increment,'
                    ' exp_id SMALLINT unsigned,'
                    ' time timestamp,'
                    ' level TINYINT,'
                    ' node varchar(100),'
                    ' msg varchar(2000) )'
                    )

        cur.execute('describe logs')
        print(cur.fetchall())

        self._logger.debug("create table raw_data")

        cur.execute('create table if not exists raw_data'
                    ' (data_id bigint unsigned primary key not null auto_increment,'
                    ' exp_id  SMALLINT unsigned,'
                    ' time timestamp,'
                    ' sensor_id SMALLINT unsigned,'
                    ' data double)'
                    )

        cur.execute('describe raw_data')
        print(cur.fetchall())

        self._logger.debug("create table sensors")

        cur.execute('create table if not exists sensors'
                    '(sensor_id SMALLINT unsigned primary key not null,'
                    'name varchar(100),'
                    'type varchar(100),'
                    'units varchar(100),'
                    'prec double,'
                    'description varchar(1000),'
                    'status varchar(100)'
                    ')')

        cur.execute('describe sensors')
        print(cur.fetchall())

        self._logger.debug("create table experiments")



        cur.execute('create table if not exists experiments'
                    '( exp_id SMALLINT unsigned primary key not null,'
                    'start_date timestamp,'
                    'end_date timestamp,'
                    'params varchar(1000)'
                    ')'
                    )

        cur.execute('describe experiments')
        print(cur.fetchall())

        self._logger.debug("create table exp_data")

        # experiment consists of search steps
        # search step consists of points

        cur.execute('create table if not exists exp_data'
                    '(point_id bigint unsigned primary key not null auto_increment,'
                    'step_id int unsigned,'    # id of search step whom this point belongs
                    'exp_id  SMALLINT unsigned,'
                    'red int,'
                    'white int,'
                    'start_time timestamp,'    # start time of dataset
                    'end_time timestamp,'     # end time of dataset
                    'number_of_data_points int unsigned,'   # num of points in set
                    'point_of_calc double,'   # point X where we calculate value of derivative
                    'f_val double,'    # value of derivative dCO2/dt(X) on this dataset
                    'q_val double,'
                    'is_finished SMALLINT unsigned '  # flag to mark if point was finished correctly and how
                    ')'
                    )
        # finished SMALLINT unsigned'
        # 0 - finished correctly
        # 1 - not finished
        #

        cur.execute('describe exp_data')
        print(cur.fetchall())

        cur.execute('commit')
        print(cur.fetchall())


if __name__ == "__main__":
    MYSQLDataSaver()



