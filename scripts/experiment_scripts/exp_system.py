#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import numpy as np
from  scipy.optimize import curve_fit
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
from exp_units_conversions import red_far_by_curr, white_far_by_curr, dry_intQ


def exp_approximation(co2, times):
    # approximation

    def exp_func(tt, a, b):
        return a * np.exp(b * tt)

    def exp_deriv(tt, a, b):
        return a*b*np.exp(b*tt)

    def lin_func(tt, a, b):
        return a * tt + b

    y = np.array(co2, dtype=float)
    x = np.array(times, dtype=float)
    # x = np.arange(0, len(y))  # ne nu eto srazu ban
    epopt, epcov = curve_fit(exp_func, x, y, p0=(2, -1)) # p0=(2.5, -1.3)
    lpopt, lepcov = curve_fit(lin_func, x, y, p0=(-2, 1))
    # print('fit exp: a={:.4f}, b={:.6f}'.format(epopt[0], epopt[1]))
    # print('fit lin: a={:.4f}, b={:.6f}'.format(lpopt[0], lpopt[1]))
    # y_eopt = exp_func(x, *epopt)
    # y_lopt = lin_func(x, *lpopt)

    # point for derivative nov is in middle of cutted time interval
    t_derivative = int(len(x)/2)

    F_lin = lpopt[0]
    F_exp = exp_deriv(t_derivative, *epopt)

    return F_lin, F_exp



class TableSearchHandler(object):
    """

    """

    def __init__(self, search_table, db_params, exp_params, logger):
        # table with rows like {"number": 100, "red": 130, "white": 130, "finished": 10}
        self._default_search_table = search_table
        self._db_params = db_params
        # same table with rows like {"number": 100, "red": 130, "white": 130, "finished": 10}
        # but with marks in "finished" field
        self._todays_search_table = deepcopy(self._default_search_table)
        self._really_calculaded_today_points = None
        self._current_point_on_calculation = None
        self._exp_id = exp_params["experiment_number"]
        self._db_name = "experiment" + str(self._exp_id)
        self._co2_sensor_id = 3  # todo fix it
        self._lamp_h = int(exp_params["lamp_h"])
        self._lamp_type = str(exp_params["lamp_type"])

        self._logger = logger

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
        # if we are here, it means that we have to find point in table with best param Q

        # set flag that we are finished
        self._last_finished_day = datetime.datetime.now().date()

        con = pymysql.connect(host=self._db_params["host"],
                              user=self._db_params["user"],
                              password=self._db_params["password"],
                              # db='experiment',
                              charset='utf8mb4',
                              cursorclass=pymysql.cursors.DictCursor)

        cur = con.cursor()
        cur.execute("use {}".format(self._db_name))

        # in db we store at least two rows with same step_id
        # lets load them for every point in _todays_search_table and find mean Q value
        # also we will bubble search point with maximum of mean Q-value
        min_point = None
        for point in self._todays_search_table:

            comm_str = "select * from exp_data where date(end_time) = date(now())" \
                       " and exp_id={} and step_id={};".format(
                self._exp_id, point['number']
            )
            cur.execute(comm_str)
            rows = cur.fetchall()

            # lets get mean sum of q_val for that two rows
            q1 = rows[0]['q_val']
            q2 = rows[1]['q_val']

            mean_q = (q1+q2)/2

            # add that value to point as new key-value pair
            point.update({'mean_q' : mean_q})

            if not min_point:
                # if it is first iteration - set first point as min
                min_point = point
            else:
                # compare values of current point and max point
                if point['mean_q'] < min_point['mean_q']:
                    min_point = point

        self._last_optimal_point = min_point
        red = self._last_optimal_point['red']
        white = self._last_optimal_point['white']

        self._logger.info("we found optimal point, it is:\n {}".format(self._last_optimal_point))

        # self._last_optimal_point = {"number": 100, "red": 121, "white": 121, "finished": 10}
        # red = self._last_optimal_point['red']
        # white = self._last_optimal_point['white']
        p_id = 65536
        # we can set any p_id because that points will not be added to db
        return [p_id, red, white]

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
        cur.execute("use {}".format(self._db_name))
        comm_str = "select point_id from exp_data order by point_id desc limit 1;"
        self._logger.debug("comm_str: {}".format(comm_str))
        cur.execute(comm_str)
        rows = cur.fetchall()
        if len(rows) != 0:
            previous_p_id = rows[0]['point_id']

            # finally
            self._current_point_id = previous_p_id + 1
        else:
            self._current_point_id = 1

        return [ self._current_point_id,
               self._current_point_on_calculation['red'],
               self._current_point_on_calculation['white'],
            self._current_point_on_calculation['led_delay'] ]  # returns list

    def calculate_next_point(self):
        if self._today != datetime.datetime.now().date():
            self._logger.info("there is new day, lets update search table")
            # check if it is time to update self._todays_search_table
            self._update_calculated_points()
            self._today = datetime.datetime.now().date()

        if datetime.datetime.now().date() == self._last_finished_day:
            self._logger.info("calc_next_point : we have already finished today")
            # it means that today`s work finished,
            # just return stored optimal point of red, white, point_id, step_id
            red = self._last_optimal_point['red']
            white = self._last_optimal_point['white']
            p_id = 65536
            mode = 'no_co2'
            led_delay = self._last_optimal_point['led_delay']
            # we can set any p_id because that points will not be added to db
            return mode, p_id, red, white, led_delay

        else:
            # we have to work a little bit more today
            # so mode is 'co2'
            mode = 'co2'
            #  lets find new random point from
            # search_rows, which have 0 in finished field
            # if there are no such rows, then lets search in search_rows, which have 1
            # if all rows have >= 2
            # then lets set flag: "self._today_is_finished "find row with best q_val ans set it as

            zero_finished = [r for r in self._todays_search_table if r['finished'] == 0]
            one_finished = [r for r in self._todays_search_table if r['finished'] == 1]
            if len(zero_finished) != 0:
                self._logger.debug("found point with 0 calculations")
                return tuple([mode] + self._prepare_random_point(zero_finished))
            elif len(one_finished) != 0:
                self._logger.debug("not found any points with 0 calculations")
                self._logger.debug("found point with 1 calculations")
                return tuple([mode] + self._prepare_random_point(one_finished))
            else:
                self._logger.debug("not found any points with 0 or 1 calculations")
                # it means that we just now finished, so we dont need to calculate co2 again
                mode = 'no_co2'
                return tuple([mode] + self._calculate_optimal_point())

        #[p for p in self._todays_search_table
                 #   if p['number'] == self._last_optimal_value_id][0]

        pass

    def update_point_data(self, p_id, t_start, t_stop):
        # if we have received this command, it means that one search point done
        # we have to save data about this point to db

        # but first lets check if we already finished
        if datetime.datetime.now().date() == self._last_finished_day:
            self._logger.info("update_point_data : we have already finished today")
        else:

            # note mb we have to do it not here, but in future

            # convert times to str
            start_time_ = datetime.datetime.fromtimestamp(
                t_start.to_sec()).strftime('%Y_%m_%d %H:%M:%S')
            self._logger.debug("start_time_: {}".format(start_time_))

            stop_time_ = datetime.datetime.fromtimestamp(
                t_stop.to_sec()).strftime('%Y_%m_%d %H:%M:%S')
            self._logger.debug("stop_time_: {}".format(stop_time_))

            # lets differentiate points from raw_data db and get f ang q
            f, q, number_of_points, is_finished = self._differentiate_co2_point(start_time_, stop_time_)

            # todo: do we really need it?
            if q < 0:
                # set status of exp point as error
                # in normal situation q everytime > 0
                self._logger.warning("calculated Q < 0, it is bad")
                is_finished = 11

            con = pymysql.connect(host=self._db_params["host"],
                                  user=self._db_params["user"],
                                  password=self._db_params["password"],
                                  # db='experiment',
                                  charset='utf8mb4',
                                  cursorclass=pymysql.cursors.DictCursor)

            cur = con.cursor()

            cur.execute("use {}".format(self._db_name))

            comm_str = 'insert into exp_data' \
                       '( point_id, step_id, exp_id, red, white, start_time, end_time,' \
                       'number_of_data_points, f_val, q_val, is_finished)' \
                       'values("{}", "{}","{}", "{}", "{}", "{}","{}", "{}", "{}", "{}", "{}")'.format(
                self._current_point_id,
                self._current_point_on_calculation['number'],
                self._exp_id,
                self._current_point_on_calculation['red'],
                self._current_point_on_calculation['white'],
                start_time_,
                stop_time_,
                number_of_points,
                f,
                q,
                is_finished
                )

            self._logger.debug("comm_str: {}".format(comm_str))  # TODO: remove after debug
            try:
                cur.execute(comm_str)
            except Exception as e:
                exc_info = sys.exc_info()
                err_list = traceback.format_exception(*exc_info)
                self._logger.error("Error while saving exp point to exp_data table: {}".format(e))
                self._logger.error(err_list)

            cur.execute('commit')
            con.close()

            # TODO do we need it really?
            self._update_calculated_points()

    def _differentiate_co2_point(self, t1, t2):

        con = pymysql.connect(host=self._db_params["host"],
                              user=self._db_params["user"],
                              password=self._db_params["password"],
                              # db='experiment',
                              charset='utf8mb4',
                              cursorclass=pymysql.cursors.DictCursor)

        cur = con.cursor()

        cur.execute("use {}".format(self._db_name))
        # TODO fix!!!!

        # for now we will handle one point differentiation in this callback
        # select time, data from raw_data where sensor_id = 3 and time
        comm_str = "select time, data from raw_data where exp_id = {} and sensor_id = {} " \
                   "and time > '{}' and time < '{}'".format(
            self._exp_id, self._co2_sensor_id, t1, t2)

        self._logger.debug("comm_str: {}".format(comm_str))  # TODO: remove after debug

        try:
            resp = cur.execute(comm_str)

            rows = cur.fetchall()

            # then lets find which rows correspond to search_table
            # for db_row in rows:

            self._logger.info(len(rows))

            # self._logger.debug(len(resp))
            co2_array = [x['data'] for x in rows]
            time_array = [x['time'] for x in rows]

            con.close()

            number_of_points = len(rows)  # todo mb we have to do filtration or smth

            if number_of_points == 0:
                is_finished = 10  # thats kinda error code: no co2 data
                f_val = 0
                q_val = 0
            else:

                converted_time = [(t - time_array[0]).total_seconds() for t in time_array]

                # cut ~ first x points before local maximum of y
                # we have to cut first x points because there are transients in co2 measurements
                # TODO check that parameter

                # lets find x in co2 array with max y value
                max_co2_position = np.argmax(co2_array)
                cut_co2 = co2_array[max_co2_position:]
                cut_time = converted_time[max_co2_position:]
                if len(cut_co2) <= 10:  # less than 10 points sucks
                    # there are too few points
                    is_finished = 12  # thats kinda error code: too few points after max value
                    f_val = 0
                    q_val = 0
                    # hh

                else:
                    # cut_time = converted_time[80:]
                    # cut_co2 = co2_array[80:]
                    cut_converted_time = [t - cut_time[0] for t in cut_time]

                    f_lin, f_exp = exp_approximation(cut_co2, cut_converted_time)
                    self._logger.info("f_lin = {}, f_exp = {}".format(f_lin, f_exp))

                    # dC - first derivative of co2 concentration in ppnmv/sec
                    # E - light intencity im mkmoles/m2*sec
                    # dT - time period of measure im sec

                    dC = -1*f_lin  # we prefer linear because it is more stable
                    E = white_far_by_curr(self._current_point_on_calculation['white'],
                        self._lamp_type, self._lamp_h) + red_far_by_curr(
                        self._current_point_on_calculation['red'], self._lamp_type, self._lamp_h)
                    # dT = (time_array[len(time_array) - 1] - time_array[0]).total_seconds()
                    dT = 900.0  # full time of one search step

                    dry_q = dry_intQ(dC, E, dT)

                    self._logger.info("dry_Q = {}".format(dry_q))

                    # print(len(cut_time))
                    # print(len(cut_co2))

                    # todo do real differentiation here
                    f_val = -1*f_lin
                    q_val = dry_q
                    is_finished = 0  # success flag

            return f_val, q_val, number_of_points, is_finished

        except Exception as e:
            # self._logger.error("Error while requesting co2 data from raw_data:")
            exc_info = sys.exc_info()
            err_list = traceback.format_exception(*exc_info)
            self._logger.error("Error while requesting co2 data from raw_data: {}".format(err_list))
            con.close()

            return 0, 0, 0, 100  # TODO check it

    def _update_calculated_points(self):
        # get all points from exp_data table which was finished today
        # mark in self._todays_search_table if some point already done
        self._logger.info("updating calculated points")
        # clear current table
        self._todays_search_table = deepcopy(self._default_search_table)

        con = pymysql.connect(host=self._db_params["host"],
                              user=self._db_params["user"],
                              password=self._db_params["password"],
                              # db='experiment',
                              charset='utf8mb4',
                              cursorclass=pymysql.cursors.DictCursor)

        cur = con.cursor()

        cur.execute("use {}".format(self._db_name))

        comm_str = 'select * from exp_data ' \
                   'where date(end_time)=date(now()) ' \
                   'and exp_id={} and is_finished=0'.format(
            self._exp_id)

        self._logger.debug("comm_str: {}".format(comm_str))

        cur.execute(comm_str)

        rows = cur.fetchall()

        # then lets find which rows correspond to search_table
        for db_row in rows:
            # self._logger.debug(db_row)
            for search_row in self._todays_search_table:
                if db_row['step_id'] == search_row['number']:
                    search_row['finished'] += 1
                    self._logger.debug(search_row)


        # then lets find new random point from
        # search_rows, which have 0 in finished field
        # if there are no such rows, then lets search in search_rows, which have 1
        # if all rows have >= 2
        # then lets set flag: "self._today_is_finished "find row with best q_val ans set it as

            # self._logger.debug(i["f_val"], i["red"], i["white"])

        # cur.execute('commit')
        con.close()

        return rows


class OrderedTableSearchHandler(TableSearchHandler):
    """
    same table search handler but use points from search table only one time
    and in strict order, and when go to the end of table - starts it again
    """

    def __init__(self, search_table, db_params, exp_params, logger):
        super(OrderedTableSearchHandler, self).__init__(search_table, db_params, exp_params, logger)
        # self.current_


    def calculate_next_point(self):
        if self._today != datetime.datetime.now().date():
            self._logger.info("there is new day, lets update search table")
            # check if it is time to update self._todays_search_table
            self._update_calculated_points()
            self._today = datetime.datetime.now().date()

        if datetime.datetime.now().date() == self._last_finished_day:
            self._logger.info("calc_next_point : we have already finished today")
            # it means that today`s work finished,
            # just return stored optimal point of red, white, point_id, step_id
            red = self._last_optimal_point['red']
            white = self._last_optimal_point['white']
            p_id = 65536
            mode = 'no_co2'
            led_delay = self._last_optimal_point['led_delay']
            # we can set any p_id because that points will not be added to db
            return mode, p_id, red, white, led_delay

        else:
            # we have to work a little bit more today
            # so mode is 'co2'
            mode = 'co2'
            #  lets find next point from
            # search_rows, which have 0 in finished field
            # if all rows have >= 1
            # then lets set flag: "self._today_is_finished "find row with best q_val ans set it as

            zero_finished = [r for r in self._todays_search_table if r['finished'] == 0]
            # one_finished = [r for r in self._todays_search_table if r['finished'] == 1]
            if len(zero_finished) != 0:
                self._logger.info("found point with 0 calculations")
                return tuple([mode] + self._prepare_random_point(zero_finished))
            # elif len(one_finished) != 0:
            #     self._logger.debug("not found any points with 0 calculations")
            #     self._logger.debug("found point with 1 calculations")
            #     return tuple([mode] + self._prepare_random_point(one_finished))
            else:
                self._logger.debug("not found any points with 0 calculations")
                # it means that we just now finished, so we dont need to calculate co2 again
                mode = 'no_co2'
                return tuple([mode] + self._calculate_optimal_point())

    def _prepare_random_point(self, list_of_points):

        con = pymysql.connect(host=self._db_params["host"],
                              user=self._db_params["user"],
                              password=self._db_params["password"],
                              # db='experiment',
                              charset='utf8mb4',
                              cursorclass=pymysql.cursors.DictCursor)

        cur = con.cursor()
        cur.execute("use {}".format(self._db_name))
        comm_str = "select point_id from exp_data order by point_id desc limit 1;"
        self._logger.debug("comm_str: {}".format(comm_str))
        cur.execute(comm_str)
        rows = cur.fetchall()
        if len(rows) != 0:
            previous_p_id = rows[0]['point_id']

            # finally
            self._current_point_id = previous_p_id + 1
        else:
            self._current_point_id = 1

        self._current_point_on_calculation = list_of_points[0]
        red = self._current_point_on_calculation ["red"]
        white = self._current_point_on_calculation ["white"]
        led_delay = self._current_point_on_calculation ["led_delay"]
        return [self._current_point_id , red, white, led_delay]
    
    def _calculate_optimal_point(self):
        last_point = self._todays_search_table[-1]
        p_id = 65536
        red = last_point["red"]
        white = last_point["white"]
        led_delay = last_point["led_delay"]
        return [p_id, red, white, led_delay]




class ExpSystemServer(object):
    """
    ros wrapper for handling exp points
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
        # self._stand_type = rospy.get_param('~exp_lamp_type', 'exp')
        # self._height_of_lamps = rospy.get_param('~exp_lamp_h', 25)

        self._search_table = rospy.get_param('exp_search_table')
        self._exp_description = rospy.get_param('mysql_data_saver_experiment_description')
        self._exp_id = self._exp_description["experiment_number"]
        self._db_params=rospy.get_param('mysql_db_params')
        self._db_name = "experiment" + str(self._exp_id)
        # self._exp_config_path = rospy.get_param('~exp_config_path', 'test.xml')
        # todo add here parsing params of gradient search and other smart methods

        # create log topic publisher
        self._log_pub = rospy.Publisher(self._log_node_name, String, queue_size=10)

        # logger
        self._logger = CustomLogger(name=self._logname, logpub=self._log_pub)
        self._logger.info("exp_server init")

        # create exp_db if it is not exists now
        self._create_exp_db()

        # create search method handler object
        # it must not contain any ros api, because i hate ros
        if self._mode == 'table':
            self._search_handler = TableSearchHandler(
                self._search_table, self._db_params, self._exp_description, self._logger)
        elif self._mode == 'ordered_table':
            self._search_handler = OrderedTableSearchHandler(
                self._search_table, self._db_params, self._exp_description, self._logger)

        # todo add other search modes
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

        # self._logger.debug("+++++++++++++++++++++++++++++++++++++++++++++++")
        # self._logger.debug("+++++++++++++++++++++++++++++++++++++++++++++++")
        self._logger.debug("we got request: {}".format(req))

        if not req.command:
            # if we got empty string
            resp = self._error_response + 'empty_command'
            return resp

        elif req.command == 'set_point_data':
            # TODO fix

            try:
                self._set_point_data(req.point_id, req.start_time, req.end_time)
                resp = ExpSystemResponse()
                resp.response = self._success_response
                self._logger.info("we got set_point_data reqv. data from control:")
                start_time_ = datetime.datetime.fromtimestamp(
                    req.start_time.to_sec()).strftime('%Y_%m_%d %H:%M:%S')
                end_time_ = datetime.datetime.fromtimestamp(
                    req.end_time.to_sec()).strftime('%Y_%m_%d %H:%M:%S')
                self._logger.info("t_start={} t_stop={}".format(
                    start_time_, end_time_
                ))
                return resp
            except Exception as e:
                exc_info = sys.exc_info()
                err_list = traceback.format_exception(*exc_info)
                self._logger.error("Service call failed: {}".format(err_list))
                resp = ExpSystemResponse()
                resp.response = self._error_response + e.args[0]
                return resp

        elif req.command == 'get_current_point':
            # request from control_node to get current point_id
            try:
                mode, p_id, red, white, led_delay = self._get_current_point()
                resp = ExpSystemResponse()
                resp.response = mode
                resp.point_id = p_id
                resp.red = red
                resp.white = white
                resp.led_delay = led_delay
                self._logger.info("we got get_current_point reqv from control:mode={} p_id={} red={} white={} delay={}".format(
                    mode, p_id, red, white, led_delay
                ))

                return resp
            except Exception as e:
                exc_info = sys.exc_info()
                err_list = traceback.format_exception(*exc_info)
                self._logger.error("Service call failed: {}".format(err_list))
                resp = ExpSystemResponse()
                resp.response = self._error_response + e.args[0]
                # resp = self._error_response + e.args[0]
                return resp

        else:
            resp = ExpSystemResponse()
            resp.response = self._error_response + 'unknown command'
            return resp

    # def _get_last_search_point_from_db(self):
    #     """
    #     sends request to mariadb to get last row in exp_data table , corresponds to exp_id
    # WE NEED ALL SEARCH POINTS FROM THIS EXP because they were used randomly
    # and we dont know how exactly
    #     """
    #     pass

    # def _load_done_points_from_db(self):
    #     # search method must do it by itself, because it depends very much
    #     # anyway use : to find points calculated today
    #     # SELECT * FROM raw_data where dayofmonth(time)=dayofmonth(now());
    #
    #
    #
    #     pass

    # def _get_point_from_db_by_id(self, point_id):
    #     pass

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
        # self._logger.debug("type(start_time): {}".format(type(start_time)))

        self._search_handler.update_point_data(p_id, start_time, stop_time)

    def _get_current_point(self):

        return self._search_handler.calculate_next_point()

        # return (random.randint(0,100), 120, 55)

    def _create_exp_db(self):

        con = pymysql.connect(host=self._db_params["host"],
                              user=self._db_params["user"],
                              password=self._db_params["password"],
                              # db='experiment',
                              charset='utf8mb4',
                              cursorclass=pymysql.cursors.DictCursor)

        cur = con.cursor()

        cur.execute("use {}".format(self._db_name))


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
        self._logger.debug(cur.fetchall())

        cur.execute('commit')
        self._logger.debug(cur.fetchall())


# def test_table_search():
#     exp_d = {
#         "experiment_type": 'table',
#         "machine_id": 'farmer4',
#         "experiment_legend": 'test table-search experiment with IRGA',
#         "start_datetime": '2020_10_18 23:00:00',
#         "end_datetime": '2020_10_30 23:00:00',
#         "additional_notes": 'add any notes about experiment errors or smth other important',
#         "experiment_id": '2020_10_18_TESTTYPE_0002',
#         "experiment_number": '2'
#     }
#     # local test
#     search_table = [
#         {"number": 1, "red": 130, "white": 130, "finished": 0},
#         {"number": 2, "red": 70, "white": 190, "finished": 0},
#         {"number": 3, "red": 190, "white": 70, "finished": 0},
#         {"number": 4, "red": 40, "white": 160, "finished": 0},
#         {"number": 5, "red": 160, "white": 40, "finished": 0},
#         {"number": 6, "red": 100, "white": 100, "finished": 0},
#         {"number": 7, "red": 220, "white": 220, "finished": 0},
#         {"number": 8, "red": 25, "white": 235, "finished": 0},
#         {"number": 9, "red": 145, "white": 115, "finished": 0},
#         {"number": 10, "red": 85, "white": 55, "finished": 0},
#         {"number": 11, "red": 205, "white": 175, "finished": 0},
#         {"number": 12, "red": 55, "white": 85, "finished": 0},
#         {"number": 13, "red": 175, "white": 205, "finished": 0},
#         {"number": 14, "red": 115, "white": 145, "finished": 0},
#         {"number": 15, "red": 235, "white": 25, "finished": 0},
#         {"number": 16, "red": 17, "white": 138, "finished": 0}
#     ]
#     db = {
#         "host": '10.9.0.12',
#         "user": '',
#         "db": 'experiment',
#         "password": ""
#     }
#     sh = TableSearchHandler(search_table, db, exp_d)
#     print(sh.calculate_next_point())

if __name__ == "__main__":
    ExpSystemServer()
    # t1 = time.time()
    # test_table_search()
    # print(time.time() - t1)
