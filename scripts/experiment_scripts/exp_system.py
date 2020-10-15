#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from std_msgs.msg import String
import traceback
import sys
from custom_logger import CustomLogger
from ros_farmer_pc.srv import ExpSystem, ExpSystemResponse


class TableSearchHandler(object):
    """

    """

    def __init__(self, path_to_config):
        # TODO fix to read xml file and parse it correctly
        import search_config_default
        self.search_table = search_config_default.search_table

    def calculate_next_point(self):
        pass



class ExpSystemServer(object):
    """
    heh
    """
    def __init__(self):
        # hardcoded constants
        self._success_response = "success: "
        self._error_response = "error: "

        # start node
        rospy.init_node('exp_server', log_level=rospy.DEBUG)

        # get roslaunch params

        # names for self topics
        self._logname = rospy.get_param('~exp_log_name', 'exp_system')
        self._log_node_name = rospy.get_param('~exp_log_node_name', 'exp_log_node')
        self._service_name = rospy.get_param('~exp_service_name', 'exp_system')
        self._exp_pub_name = rospy.get_param('~exp_pub_name', 'exp_pub')

        # mode of work
        # no matter which method is chosen we will parse params for all of them from .launch file
        self._mode = rospy.get_param('~exp_mode_name', 'table')
        self._exp_config_path = rospy.get_param('~exp_config_path', 'test.xml')
        # todo add here parsing params of gradient search and other smart methods

        if self._mode == 'table':
            self._search_handler = TableSearchHandler(self._exp_config_path)

        # todo add other search modes

        # create log topic publisher
        self._log_pub = rospy.Publisher(self._log_node_name, String, queue_size=10)
        # create data topic publisher
        # self._co2_pub = rospy.Publisher(self._raw_co2_pub_name, Temperature, queue_size=10)

        # logger
        self._logger = CustomLogger(name=self._logname, logpub=self._log_pub)
        self._logger.info("exp_server init")

        # service
        self._service = rospy.Service(self._service_name, ExpSystem, self._handle_request)

    def _handle_request(self, req):

        self._logger.debug("+++++++++++++++++++++++++++++++++++++++++++++++")
        self._logger.debug("+++++++++++++++++++++++++++++++++++++++++++++++")
        self._logger.debug("we got request: {}".format(req))

        if not req.command:
            # if we got empty string
            resp = self._error_response + 'empty_command'
            return resp

        elif req.command == 'put_exp_data':
            # TODO fix

            try:
                self._put_exp_data(req)
                resp = ExpSystemResponse()
                resp.response = self._success_response
                return resp
            except Exception as e:
                exc_info = sys.exc_info()
                err_list = traceback.format_exception(*exc_info)
                self._logger.error("Service call failed: {}".format(err_list))
                resp = self._error_response + e.args[0]
                return resp

        elif req.command == 'get_light_mode':
            # TODO fix
            # TODO mb we need command to start generate new point?
            # it means that it is time to get new
            try:
                self._get_light_mode(req)
                resp = ExpSystemResponse()
                # resp.response = self._success_response
                # resp.point_id =
                #req.
                return resp
            except Exception as e:
                exc_info = sys.exc_info()
                err_list = traceback.format_exception(*exc_info)
                self._logger.error("Service call failed: {}".format(err_list))
                resp = self._error_response + e.args[0]
                return resp

        else:
            resp = self._error_response + 'unknown command'
            return resp

    def _get_point_from_db_by_id(self, point_id):
        pass

    def _put_exp_data(self, req):
        # this command handles incoming msg with data about experiment
        # we want to get from this msg
        # uint32 point_id
        # time start_time
        # time end_time
        # and put that to correspond row in mysql db
        #
        pass

    def _start_calculating(self, req):
        # when we got this command, we have to start calculate new search point
        pass


    def _get_light_mode(self, req):
        pass

    def _calculate_new_point(self):
        pass