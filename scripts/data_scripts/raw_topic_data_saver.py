#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import datetime
from std_msgs.msg import String
from data_scripts.custom_logger import CustomLogger
from ros_farmer_pc.srv import ControlSystem, LedDevice, RelayDevice, SBA5Device, SBA5DeviceResponse
from sensor_msgs.msg import Temperature
from threading import RLock, Event
import time
import numpy as np
import h5py
import re
import os.path


class DataSaverException(Exception):
    pass


class TopicSubHandler(object):
    """
    handle one raw data topic
    and its callback
    """
    def __init__(self, topic_name_, data_lock_, msg_type_=None, ):
        if not msg_type_:
            self._msg_type = Temperature

        self._topic_name = topic_name_
        self._data_lock = data_lock_

    def callback(self, msg):

        # must be threadsafe!
        pass

class DataSaverServer(object):
    """
    it can
    - find all raw data topics
    - subscribe to them
    - create hdf5-files for every raw data topic
    - when there is new message in topic, we parse it, add normal time, date and
    info about experiment.
    after
    """

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
        #

    def __init__(self):
        # hardcoded constants
        self._success_response = "success: "
        self._error_response = "error: "

        # start node
        rospy.init_node('data_saver_server')

        # get roslaunch params

        # names for self topics
        self._logname = rospy.get_param('~data_saver_log_name', 'data_saver')
        self._log_node_name = rospy.get_param('~data_saver_log_node_name', 'data_saver_log')

        # create log topic publisher
        self._log_pub = rospy.Publisher(self._log_node_name, String, queue_size=10)

        # start logger
        self._logger = CustomLogger(name=self._logname, logpub=self._log_pub)
        self._logger.debug("data_saver server init")

        #self._service_name = rospy.get_param('~data_saver_service_name', 'data_saver_system')
        #self._raw_co2_pub_name = rospy.get_param('~control_raw_co2_pub_name', 'raw_co2_pub')

        # devices services names
        #self._relay_service_name = rospy.get_param('~control_relay_service_name', 'relay_device')
        #self._led_service_name = rospy.get_param('~control_led_service_name', 'led_device')
        #self._sba5_service_name = rospy.get_param('~control_sba5_service_name',
        #                                          'sba5_device')
        # we are using sba5 by default co2 sensor

        # get list of all topics to save data from
        # raw_data_names_string = """
        # /bmp180_1_pressure_pub
        # /bmp180_1_temp_pub
        # /raw_co2_pub
        # /relay_1_sub
        # /si7021_1_hum_pub
        # /si7021_1_temp_pub
        # """
        # logs_names_string="""
        # /control_log_node
        # /led_log_node
        # /relay_log_node
        # /sba5_log_node
        # """
        default_id = self.generate_experiment_id()
        self._experiment_id = rospy.get_param('~data_saver_experiment_id', default_id)
        # experiment id is string like that YYYY_MM_DD_TYPE_NUMB
        # where YYYY_MM_DD is data of start
        # TYPE is short type of experiment (like TEST, TABLE, SEARCH etc)
        # NUMB is uniq number of experiment

        # lets create lock to allow only one callback-thread to write to hdf5 file
        self._data_write_lock = RLock()

        # check fields of hdf datafile
        self._data_path = rospy.get_param('~data_saver_data_folder', './data/raw_data.hdf5')
        self._raw_topics_string = rospy.get_param(
        '~data_saver_raw_topics_string',
        '/bmp180_1_temp_pub,/bmp180_1_pressure_pub,'
        '/raw_co2_pub,/si7021_1_hum_pub,/si7021_1_temp_pub'
        )
        # parse that string
        self._topics_list = self._raw_topics_string.split(",")

        # list to keep subscribers
        self._subscribers_list = list()
        # TODO: here
        # check if datafile exists, or update it
        self._create_hdf5_file()


        # sub to all topics from parsed string
        for topic in self._topics_list:
            # important!
            # subscribe to all topics with one callback
            # it is not good
            # or it is good?

            # we have to dynamically create callback-handler objects
            # TODO: add objects
            # for now we think that all raw_data_topics have type "sensor_msgs/Temperature"
            # it must be in architecture of all system !
            # so here we think that it is "sensor_msgs/Temperature" as default
            s = rospy.Subscriber(name=topic, data_class=Temperature,
                             callback=self._raw_data_callback, callback_args=None,
                 queue_size=10)
            self._subscribers_list.append(s)

        # =================== planning is here =================================
        # create hdf5 with meta
        # do create_dataset to every topic
        # when data from data_topic
        # just :
        # with h5py.File('exp_test.hdf5', 'a') as f:
        #     dset = f['dataset']
        #     dset_time_len = np.shape(dset)[1]
        #     dset.resize((2, dset_time_len + 1))
        #     dset[:, np.shape(dset)[1] - 1] = np.array(new_data)

        # how then read data for calculating F, Q and other?
        # mb by timestamp
        # like:

    def _raw_data_callback(self, data_message):
        # TODO: do !
        #  with threading locks
        with self._data_write_lock:
            pass

    def _create_hdf5_file(self):
        # if there is no file we have to create it in a mode

        with h5py.File(self._data_path, 'a') as f:
            try:
                g = f.create_group('Raw_Data')
            except ValueError as v:
                # it means we already have that group
                # so just skip that step
                self._logger.warning("error while creating hdf data file: {}".format(v))

            # then find this group
            raw_data = f['Raw_Data']

            for topic in self._topics_list:
                try:
                    # we have to create dataset for every data topic, whose name we got
                    raw_data.create_dataset(topic, (2, 1), maxshape=(2, None), dtype='float64')
                except ValueError as v:
                    # it means we already have that dataset
                    # so just skip that step
                    self._logger.warning("error while creating hdf data file: {}".format(v))

            #d = g.create_dataset('default', data=arr)
            # d = g.create_dataset('dataset', (2, 1), maxshape=(2, None), dtype='float64')

            # then lets check what have we done
            self._logger.debug("lets show all entities in new hdf file: ")

            # cringe
            def get_all(name):
                self._logger.debug(name)
            # strange thing
            f.visit(get_all)

            # then we have to create metadata to this file
            # TODO add correct metadata and check it

        pass
