#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import datetime
from std_msgs.msg import String, Header
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
        rospy.init_node('data_saver_server', log_level=rospy.DEBUG)

        # get roslaunch params

        # names for self topics
        self._logname = rospy.get_param('~data_saver_log_name', 'data_saver')
        self._log_node_name = rospy.get_param('~data_saver_log_node_name', 'data_saver_log')

        # create log topic publisher
        self._log_pub = rospy.Publisher(self._log_node_name, String, queue_size=10)

        # start logger
        self._logger = CustomLogger(name=self._logname, logpub=self._log_pub)
        self._logger.debug("data_saver server init")


        default_id = self.generate_experiment_id()
        self._experiment_id = rospy.get_param('~data_saver_experiment_id', default_id)
        # experiment id is string like that YYYY_MM_DD_TYPE_NUMB
        # where YYYY_MM_DD is data of start
        # TYPE is short type of experiment (like TEST, TABLE, SEARCH etc)
        # NUMB is uniq number of experiment

        # lets create lock to allow only one callback-thread to write to hdf5 file
        self._data_write_lock = RLock()

        # check fields of hdf datafile
        self._data_place = rospy.get_param('~data_saver_data_folder', './exp_data')
        self._data_file_name = rospy.get_param('~data_saver_data_file_name', 'raw_data.hdf5')
        self._data_path = self._data_place + '/' + self._data_file_name
        self._logger.debug(" === data path is {}".format(self._data_path))

        #  example
        #  http://wiki.ros.org/rospy/Overview/Parameter%20Server

        self._raw_topics = rospy.get_param('~data_saver_raw_topics')
        self._exp_topics = rospy.get_param('~data_saver_exp_topics')

        self._description = rospy.get_param('~data_saver_experiment_description')

        # list to keep subscribers (for what?)
        self._subscribers_list = list()

        # check if datafile exists, or update it
        self._create_hdf5_file()

        # sub to all topics from parsed string
        self._logger.debug("creating raw subs")
        for topic in self._raw_topics:
            # for now we think that all raw_data_topics have type "sensor_msgs/Temperature"
            # it must be in architecture of all system !
            # so here we think that it is "sensor_msgs/Temperature" as default
            s = rospy.Subscriber(name=topic['name'], data_class=Temperature,
                             callback=self._raw_data_callback, callback_args=topic,
                 queue_size=10)
            print(s)
            self._subscribers_list.append(s)

        self._logger.debug("creating exp subs")
        for topic in self._exp_topics:
            # for now we think that all raw_data_topics have type "sensor_msgs/Temperature"
            # it must be in architecture of all system !
            # so here we think that it is "sensor_msgs/Temperature" as default
            s = rospy.Subscriber(name=topic['name'], data_class=Temperature,
                             callback=self._raw_data_callback, callback_args=topic,
                 queue_size=10)
            print(s)
            self._subscribers_list.append(s)

        self._logger.debug("end of init, go to loop")

        self._loop()

    def _loop(self):
        rospy.spin()

        # ====== full plan:
        # load and parse config for topics
        # create hdf5 with meta
        # do create_dataset to every topic

        # when data from data_topic
        # just :
        # with h5py.File('exp_test.hdf5', 'a') as f:
        #     dset = f['dataset']
        #     dset_time_len = np.shape(dset)[1]
        #     dset.resize((2, dset_time_len + 1))
        #     dset[:, np.shape(dset)[1] - 1] = np.array(new_data)
        # =================== planning is here =================================
        # how then read data for calculating F, Q and other?
        # mb by timestamp
        # like:

    def _raw_data_callback(self, data_message, topic_info):

        # topic_info must be dict and contain 'name', 'type', 'dtype', 'units', 'status':
        # example:
        # { name: '/bmp180_1_temp_pub', type: 'temperature', dtype: 'float64', units: 'C', status:'raw },

        # example how to convert rospy time to datetime
        # (datetime.datetime.fromtimestamp(rospy.Time.now().to_sec())).strftime('%Y_%m_%d,%H:%M:%S')

        def get_items(name, obj):
            print(name, obj)

        new_data = data_message.temperature
        timestamp = data_message.header.stamp.to_sec()  # TODO check if we mb need to read nsecs too
        exp_point_id = data_message.variance

        with self._data_write_lock:
            with h5py.File(self._data_path, 'a') as f:
                # we have to add one point (data_message data and timestamp and mb poid)

                # check if it is a raw message or exp message
                if topic_info['status'] == 'raw':
                    # it is raw, we have to parse and save only 2 fields: timestamp and data
                    # TODO: handle path to raw dataset more handy way
                    self._logger.debug('/Raw_Data'+ topic_info['name'])

                    # REMEMBER: topic_name starts with '/' so
                    # group[topic_name] will raise error

                    # raw_data = f['Raw_Data']
                    dset = f['/Raw_Data'+ topic_info['name']]
                    dset_len = np.shape(dset)[1]
                    dset.resize((2, dset_len + 1))
                    dset[:, dset_len - 1] = np.array([timestamp, new_data])
                    # print("new dataset {} state : ".format(dset))
                    # print(dset[:,:])

                elif topic_info['status'] == 'exp':
                    # it is exp data, we have to parse and save  3 fields: timestamp and poid and data
                    # TODO: handle path to exp dataset more handy way
                    self._logger.debug('/Exp_Data' + topic_info['name'])
                    # exp_data = f['Exp_Data']
                    dset = f['/Exp_Data' + topic_info['name']]
                    dset_len = np.shape(dset)[1]
                    dset.resize((3, dset_len + 1))
                    dset[:, dset_len - 1] = np.array([timestamp, exp_point_id, new_data])
                    # print("new dataset {} state : ".format(dset))
                    # print(dset[:,:])


    def _create_hdf5_file(self):
        # if there is no file we have to create it in 'a' mode
        # we have to create one table for every topic from list
        # every measure will keep [timestamp, data]
        self._logger.debug("creating hdf5 file if it doesnt exist")

        # cringe
        def get_all(name):
            print(name)
            #self._logger.debug(name)

        def get_items(name, obj):
            print(name, obj)

        def get_attrs(name, obj):
            print(name, obj)
            for m in obj.attrs.keys():
                print('{} : {}'.format(m, obj.attrs[m]))


        # check existing of path and create directory if needed
        if not os.path.exists(self._data_place):
            os.makedirs(self._data_place)
            self._logger.debug("Directory {} created".format(self._data_place))
        else:
            self._logger.debug("Directory {} exists".format(self._data_place))

        # then try to check file
        self._logger.debug("final data_path is {}".format(self._data_path))
        with h5py.File(self._data_path, 'a') as f:
            try:
                g = f.create_group('Raw_Data')
            except ValueError as v:
                # it means we already have that group
                # so just skip that step
                self._logger.warning("error while creating hdf data file: {}".format(v))

            # then find this group
            raw_data = f['Raw_Data']
            self._logger.debug("raw_data is {}".format(raw_data))

            for topic in self._raw_topics:
                print(topic)
                try:
                    # we have to create dataset for every data topic, whose name we got

                    # note:
                    # if name of dataset starts with '/' like '/data2'
                    # operation group.create_dataset('/data2') will create dataset in '/'
                    # not in '/group/data2'
                    # so we have to write this path manually
                    # like

                    ds = f.create_dataset('Raw_Data'+ topic['name'], (2, 1), maxshape=(2, None), dtype=topic['dtype'])
                    self._logger.debug("raw_data is {}".format(raw_data))
                    print("new ds is {}".format(ds))
                except Exception as v:
                    # it means we already have that dataset
                    # so just skip that step
                    self._logger.warning("error while creating hdf data file: {}".format(v))

                # then we have to create metadata to all datasets of this file
                try:
                    meta = {
                        'start_date': datetime.datetime.now().strftime('%Y_%m_%d'),
                        'data_type': topic['type'],
                        'data_units': topic['units'],
                        'sensor_name': topic['name']
                    }



                    ds = f['Raw_Data'+topic['name']]
                    ds.attrs.update(meta)
                except Exception as v:
                    # it means we already have that dataset
                    # so just skip that step
                    self._logger.warning("error while adding meta to hdf data file: {}".format(v))

            # create group for exp data
            try:
                g = f.create_group('Exp_Data')
            except ValueError as v:
                # it means we already have that group
                # so just skip that step
                self._logger.warning("error while creating hdf data file: {}".format(v))

            # then find this group
            exp_data = f['Exp_Data']

            # then lets create datasets for experimental calculated data
            # every measure will keep [timestamp, search_point_id, data]

            for topic in self._exp_topics:
                print(topic)

                try:
                    # we have to create dataset for every data topic, whose name we got
                    exp_data = f['Exp_Data']
                    ds = f.create_dataset('Exp_Data'+topic['name'], (3, 1), maxshape=(3, None), dtype=topic['dtype'])
                    print("new ds is {}".format(ds))
                except Exception as v:
                    # it means we already have that dataset
                    # so just skip that step
                    self._logger.warning("error while creating hdf data file: {}".format(v))
                # then we have to create metadata to all datasets of this file
                try:
                    meta = {
                        'start_date': datetime.datetime.now().strftime('%Y_%m_%d'),
                        'data_type': topic['type'],
                        'data_units': topic['units'],
                        'exp_value_name': topic['name']
                    }
                    ds = f['Exp_Data'+ topic['name']]
                    ds.attrs.update(meta)
                except Exception as v:
                    # it means we already have that dataset
                    # so just skip that step
                    self._logger.warning("error while adding meta to hdf data file: {}".format(v))

            # global meta about all experiment
            try:
                # meta = {
                #     'start_datetime': datetime.datetime.now().strftime('%Y_%m_%dT%H:%M:%S'),
                #     'end_datetime': 'unknown',
                #     'experiment_type': 'unknown',
                #     'experiment_legend': 'unknown',
                #     'experiment_id': self._experiment_id
                #     # TODO add this params from .launch file, not from here
                # }

                meta = self._description

                f.attrs.update(meta)
            except Exception as e:
                self._logger.warning("error while adding global meta to hdf data file: {}".format(v))


            # then lets check what have we done
            self._logger.debug("lets show all entities in new hdf file: ")

            # strange thing
            f.visit(get_all)
            f.visititems(get_items)

            self._logger.debug("check meta")

            # check metadata
            f.visititems(get_attrs)

            # check metadata example
            self._logger.debug("check meta of header of file")
            for m in f.attrs.keys():
                 print('{} : {}'.format(m, f.attrs[m]))


if __name__ == "__main__":
    DataSaverServer()