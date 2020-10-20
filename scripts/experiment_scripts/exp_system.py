#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import random
import pymysql
import datetime
from std_msgs.msg import String
import traceback
import sys
import time
from copy import deepcopy
from custom_logger import CustomLogger
from ros_farmer_pc.srv import ExpSystem, ExpSystemResponse


class TableSearchHandler(object):
    """

    """

    def __init__(self, search_table, db_params, exp_params):
        self._default_search_table = search_table
        self._db_params = db_params
        self._todays_search_table = deepcopy(self._default_search_table)
        self._current_point_on_calculation = None
        self._exp_id = exp_params["experiment_number"]

        self._last_finished_day = None  # datetime.date
        self._last_optimal_point = None  # like {"number": 100, "red": 130, "white": 130, "finished": 10}

        self._current_point_on_search = None  # like {"number": 100, "red": 130, "white": 130, "finished : 10}
        self._current_point_id = None

        self._today = datetime.datetime.now().date()  # current day of calculation

        self._update_calculated_points()

        # self.con = pymysql.connect(host='localhost',
        #                       user='admin',
        #                       password='admin',
        #                       # db='experiment',
        #                       charset='utf8mb4',
        #                       cursorclass=pymysql.cursors.DictCursor)
    def _calculate_optimal_point(self):
        # TODO
        pass

    def _prepare_random_point(self, list_of_points):
        """
        get one random point from list and do all stuff
        """
        # that point will go to calculation
        self._current_point_on_calculation = random.choice(list_of_points)
        # lets generate correct p-id for it
        # it is important so ask from db

        con = pymysql.connect(host=self._db_params["host"],
                              user=self._db_params["user"],
                              password=self._db_params["password"],
                              # db='experiment',
                              charset='utf8mb4',
                              cursorclass=pymysql.cursors.DictCursor)

        cur = con.cursor()
        cur.execute("use {}".format(self._db_params["db"]))
        comm_str = "select point_id from exp_data order by end_time desc limit 1;"
        print("comm_str: {}".format(comm_str))
        cur.execute(comm_str)
        rows = cur.fetchall()
        previous_p_id = rows[0]['point_id']
        # finally
        self._current_point_id = previous_p_id + 1
        return self._current_point_id, \
               self._current_point_on_calculation['red'], \
               self._current_point_on_calculation['white']

    def calculate_next_point(self):
        if self._today != datetime.datetime.now().date():
            print("lets update search table")
            # check if it is time to update self._todays_search_table
            self._update_calculated_points()
            self._today = datetime.datetime.now().date()

        if datetime.datetime.now().date() == self._last_finished_day:
            print("we have already finished today")
            # it means that today`s work finished,
            # just return stored optimal point of red, white, point_id, step_id
            red = self._last_optimal_point['red']
            white = self._last_optimal_point['white']
            p_id = 65536
            # we can set any p_id because that points will not be added to db
            return (p_id, red, white)

        else:
            # we have to work a little bit more today

            #  lets find new random point from
            # search_rows, which have 0 in finished field
            # if there are no such rows, then lets search in search_rows, which have 1
            # if all rows have >= 2
            # then lets set flag: "self._today_is_finished "find row with best q_val ans set it as

            zero_finished = [r for r in self._todays_search_table if r['finished'] == 0]
            one_finished = [r for r in self._todays_search_table if r['finished'] == 1]
            if len(zero_finished) != 0:
                print("found point with 0 calculations")
                return self._prepare_random_point(zero_finished)
            elif len(one_finished) != 0:
                print("not found any points with 0 calculations")
                print("found point with 1 calculations")
                return self._prepare_random_point(one_finished)
            else:
                print("not found any points with 0 or 1 calculations")
                return self._calculate_optimal_point()

        #[p for p in self._todays_search_table
                 #   if p['number'] == self._last_optimal_value_id][0]

        pass

    def update_point_data(self, p_id, t_start, t_stop):
        pass

    def _update_calculated_points(self):
        # get all points from exp_data table which was finished today
        # mark in self._todays_search_table if some point already done
        print("lets update calculated points")
        # clear current table
        self._todays_search_table = deepcopy(self._default_search_table)

        con = pymysql.connect(host=self._db_params["host"],
                              user=self._db_params["user"],
                              password=self._db_params["password"],
                              # db='experiment',
                              charset='utf8mb4',
                              cursorclass=pymysql.cursors.DictCursor)

        cur = con.cursor()

        cur.execute("use {}".format(self._db_params["db"]))

        comm_str = 'select * from exp_data ' \
                   'where date(end_time)=date(now()) ' \
                   'and exp_id={} and is_finished=0'.format(
            self._exp_id)

        print("comm_str: {}".format(comm_str))

        cur.execute(comm_str)

        rows = cur.fetchall()

        # then lets find which rows correspond to search_table
        for db_row in rows:
            # print(db_row)
            for search_row in self._todays_search_table:
                if db_row['step_id'] == search_row['number']:
                    search_row['finished'] += 1
                    print(search_row)


        # then lets find new random point from
        # search_rows, which have 0 in finished field
        # if there are no such rows, then lets search in search_rows, which have 1
        # if all rows have >= 2
        # then lets set flag: "self._today_is_finished "find row with best q_val ans set it as

            # print(i["f_val"], i["red"], i["white"])

        # cur.execute('commit')
        con.close()

        return rows



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
        self._search_table = rospy.get_param('exp_search_table')
        self._exp_description = rospy.get_param('mysql_data_saver_experiment_description')
        self._exp_id = self._exp_description["experiment_number"]
        self._db_params=rospy.get_param('mysql_db_params')
        # self._exp_config_path = rospy.get_param('~exp_config_path', 'test.xml')
        # todo add here parsing params of gradient search and other smart methods

        # create exp_db if it is not exists now
        self._create_exp_db()

        # create search method handler object
        # it must not contain any ros api, because i hate ros
        if self._mode == 'table':
            self._search_handler = TableSearchHandler(
                self._search_table, self._db_params, self._exp_description)



        # todo add other search modes

        # create log topic publisher
        self._log_pub = rospy.Publisher(self._log_node_name, String, queue_size=10)
        # create data topic publisher
        # self._co2_pub = rospy.Publisher(self._raw_co2_pub_name, Temperature, queue_size=10)

        # logger
        self._logger = CustomLogger(name=self._logname, logpub=self._log_pub)
        self._logger.info("exp_server init")

        # now we have to calculate current search point
        # self._search_handler will calculate and store it
        # until we get reqv from control system
        self._search_handler.calculate_next_point()

        # service
        self._service = rospy.Service(self._service_name, ExpSystem, self._handle_request)

        self._loop()

    def _loop(self):
        rospy.spin()

    def _handle_request(self, req):

        self._logger.debug("+++++++++++++++++++++++++++++++++++++++++++++++")
        self._logger.debug("+++++++++++++++++++++++++++++++++++++++++++++++")
        self._logger.debug("we got request: {}".format(req))

        if not req.command:
            # if we got empty string
            resp = self._error_response + 'empty_command'
            return resp

        elif req.command == 'set_point_data':
            # TODO fix

            try:
                self._set_point_data(req.point_id, req.start_time, req.stop_time)
                resp = ExpSystemResponse()
                resp.response = self._success_response
                self._logger.info("we got data from control t_start={} t_stop={}".format(
                    req.start_time, req.stop_time
                ))
                return resp
            except Exception as e:
                exc_info = sys.exc_info()
                err_list = traceback.format_exception(*exc_info)
                self._logger.error("Service call failed: {}".format(err_list))
                resp = self._error_response + e.args[0]
                return resp

        elif req.command == 'get_current_point':
            # request from control_node to get current point_id
            try:
                p_id, red, white = self._get_current_point()
                resp = ExpSystemResponse()
                resp.response = self._success_response
                resp.point_id = p_id
                resp.red = red
                resp.white = white
                self._logger.info("we got reqv from control; and p_id={} red={} white={}".format(
                    p_id, red, white
                ))

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

    # def _get_last_search_point_from_db(self):
    #     """
    #     sends request to mariadb to get last row in exp_data table , corresponds to exp_id
    # WE NEED ALL SEARCH POINTS FROM THIS EXP because they were used randomly
    # and we dont know how exactly
    #     """
    #     pass

    def _load_done_points_from_db(self):
        # search method must do it by itself, because it depends very much
        # anyway use : to find points calculated today
        # SELECT * FROM raw_data where dayofmonth(time)=dayofmonth(now());
        pass

    def _get_point_from_db_by_id(self, point_id):
        pass

    def _set_point_data(self, p_id, start_time, stop_time):
        # this command handles incoming msg with data about experiment
        # we want to get from this msg
        # uint32 point_id
        # time start_time
        # time end_time
        # and put that to correspond row in mysql db
        #

        # NOTE we have to use update statement
        # https://oracleplsql.ru/update-mariadb.html
        # update raw_data set data_id=5754714, exp_id=200, time='2020-10-19 23:23:06',
        # data=500 where data_id=5754714;
        pass

    # def _start_calculating(self, req):
    #     # when we got this command, we have to start calculate new search point
    #     pass


    def _get_current_point(self):
        return (random.randint(0,100), 120, 50)


    def _calculate_new_point(self):
        pass

    def _create_exp_db(self):

        con = pymysql.connect(host=self._db_params["host"],
                              user=self._db_params["user"],
                              password=self._db_params["password"],
                              # db='experiment',
                              charset='utf8mb4',
                              cursorclass=pymysql.cursors.DictCursor)

        cur = con.cursor()

        cur.execute("use {}".format(self._db_params["db"]))

        cur.execute('create table if not exists exp_data'
                    '(point_id bigint unsigned primary key not null auto_increment,'
                    'step_id int unsigned,'  # id of search step whom this point belongs
                    'exp_id  SMALLINT unsigned,'
                    'red int,'
                    'white int,'
                    'start_time timestamp,'  # start time of dataset
                    'end_time timestamp,'  # end time of dataset
                    'number_of_data_points int unsigned,'  # num of points in set
                    'point_of_calc double,'  # point X where we calculate value of derivative
                    'f_val double,'  # value of derivative dCO2/dt(X) on this dataset
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


def test_table_search():
    exp_d = {
        "experiment_type": 'table',
        "machine_id": 'farmer4',
        "experiment_legend": 'test table-search experiment with IRGA',
        "start_datetime": '2020_10_18 23:00:00',
        "end_datetime": '2020_10_30 23:00:00',
        "additional_notes": 'add any notes about experiment errors or smth other important',
        "experiment_id": '2020_10_18_TESTTYPE_0002',
        "experiment_number": '2'
    }
    # local test
    search_table = [
        {"number": 1, "red": 130, "white": 130, "finished": 0},
        {"number": 2, "red": 70, "white": 190, "finished": 0},
        {"number": 3, "red": 190, "white": 70, "finished": 0},
        {"number": 4, "red": 40, "white": 160, "finished": 0},
        {"number": 5, "red": 160, "white": 40, "finished": 0},
        {"number": 6, "red": 100, "white": 100, "finished": 0},
        {"number": 7, "red": 220, "white": 220, "finished": 0},
        {"number": 8, "red": 25, "white": 235, "finished": 0},
        {"number": 9, "red": 145, "white": 115, "finished": 0},
        {"number": 10, "red": 85, "white": 55, "finished": 0},
        {"number": 11, "red": 205, "white": 175, "finished": 0},
        {"number": 12, "red": 55, "white": 85, "finished": 0},
        {"number": 13, "red": 175, "white": 205, "finished": 0},
        {"number": 14, "red": 115, "white": 145, "finished": 0},
        {"number": 15, "red": 235, "white": 25, "finished": 0},
        {"number": 16, "red": 17, "white": 138, "finished": 0}
    ]
    db = {
        "host": '10.9.0.12',
        "user": '',
        "db": 'experiment',
        "password": ""
    }
    sh = TableSearchHandler(search_table, db, exp_d)
    print(sh.calculate_next_point())

if __name__ == "__main__":
    # ExpSystemServer()
    t1 = time.time()
    test_table_search()
    print(time.time() - t1)
