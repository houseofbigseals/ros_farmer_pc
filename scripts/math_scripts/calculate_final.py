import numpy as np
import pylab as pl
import pymysql
import sys, traceback
from scipy.optimize import curve_fit
from exp_units_conversions import red_far_by_curr, white_far_by_curr, dry_intQ, final_intQ

def sweet_print(str_):
    print("\n{:<30} {}".format("[   NOTE   ]", str_))

def calculate_final_effect(_db_params, _search_table, _exp_id, dM_exp, dM_control,
                           R_control, W_control, h_led_control, h_led_exp):
    # get one point remotely and calculate F
    con = pymysql.connect(host=_db_params["host"],
                          user=_db_params["user"],
                          password=_db_params["password"],
                          db=_db_params['db'],
                          charset='utf8mb4',
                          cursorclass=pymysql.cursors.DictCursor)

    cur = con.cursor()



    # get days of experiment
    comm_str = "select distinct(date(end_time)) as day from exp_data where " \
               "is_finished=0 and exp_id={};".format(_exp_id)
    cur.execute(comm_str)
    rows = cur.fetchall()
    # return (_db_params, _search_table, _exp_id)
    days = [x['day'] for x in rows]
    print("days of experiment: ")
    for d in days:
        print(str(d))
    E_mean_days = []

    # ok_symb = "[ NOTE ]"

    for d in days:
        print("\n ==================================================================")
        print("try to calculate day {}".format(d))
        print(" ==================================================================")
        # search min_G point and find correspond R_min and W_min

        # in db we store at least two rows with same step_id
        # lets load them for every point in _todays_search_table and find mean Q value
        # also we will bubble search point with maximum of mean Q-value
        min_g_point = None
        max_f_point = None
        error_points = list()
        num_of_table_points = 0
        for point in _search_table:
            try:
                comm_str = "select * from exp_data where date(end_time) = date('{}')" \
                           " and exp_id={} and step_id={} and is_finished=0;".format(
                    d, _exp_id, point['number'])
                # print(comm_str)
                cur.execute(comm_str)
                rows = cur.fetchall()

                # lets get mean sum of q_val for that two rows
                q1 = rows[0]['q_val']
                q2 = rows[1]['q_val']

                mean_q = (q1 + q2) / 2

                f1 = rows[0]['f_val']
                f2 = rows[1]['f_val']

                mean_f = (f1 + f2) / 2

                # add that value to point as new key-value pair
                point.update({'mean_q': mean_q})
                point.update({'mean_f': mean_f})

                if not min_g_point:
                    # if it is first iteration - set first point as min
                    min_g_point = point
                else:
                    # compare values of current point and max point
                    if point['mean_q'] < min_g_point['mean_q']:
                        min_g_point = point

                if not max_f_point:
                    # if it is first iteration - set first point as min
                    max_f_point = point
                else:
                    # compare values of current point and max point
                    if point['mean_f'] > max_f_point['mean_f']:
                        max_f_point = point

                num_of_table_points += 1
            except Exception as e:
                exc_info = sys.exc_info()
                err_list = traceback.format_exception(*exc_info)
                print("\n ERROR, point: {} \n calculation failed: {} \n".format(point, err_list))
                error_points.append(point)

        min_g_point.update({'date': d})
        max_f_point.update({'date': d})
        sweet_print("we have calculated {} search table lines from 16".format(num_of_table_points))

        count_command = "select count(*) from exp_data where date(end_time)" \
                        " = date('{}') and exp_id={};".format(d, exp_id)
        cur.execute(count_command)
        rows = cur.fetchall()
        num_of_all_points = rows[0]['count(*)']
        sweet_print("num of problems in search table {} from 16 ".format(len(error_points)))
        sweet_print("full number of day {} stored points is {}, must be about 32 ".format(d, num_of_all_points))
        sweet_print("min g point is : {}".format(min_g_point))
        # print("\n min f point is : {} \n".format(max_f_point))

        r_min = min_g_point['red']
        w_min = min_g_point['white']

        # print("final search table:")
        # for p in _search_table:
        #     print(p)

        continue_ = str(raw_input("would you like to add this day to calculation? y or n: "))
        if continue_ == 'y':

            # second - calculation of control E_mean (in ppfd) final value for each day
            command = "select * from exp_data where date(end_time) = " \
                      "date('{}') and exp_id={};".format(d, _exp_id)
            cur.execute(command)
            rows = cur.fetchall()

            num_of_search_points = len(rows)
            num_of_stable_points = 96 - num_of_search_points
            # 96 is a number of 15-mins intervals in 24 hours
            sweet_print("in day {} we got {} search points and {} stable points".format(
                d, num_of_search_points, num_of_stable_points))

            reds = [a["red"] for a in rows ]
            whites = [a["white"] for a in rows]

            E_search_summ = 0

            for (r, w) in zip(reds, whites):
                E_search_summ += red_far_by_curr(r, "exp", h_led_exp) + \
                    white_far_by_curr(w, "exp", h_led_exp)

            E_search_mean = E_search_summ/96
            E_stable_summ = red_far_by_curr(r_min, "exp", h_led_exp) + \
                    white_far_by_curr(w_min, "exp", h_led_exp)
            E_stable_mean = E_stable_summ * num_of_stable_points / 96
            sweet_print("in day {} we got E_search_summ = {} E_search_mean = {} and E_stable_mean = {}"
                  .format(d, E_search_summ, E_search_mean, E_stable_mean))
            sweet_print("r_min_stable = {}, w_min_stable = {}".format(r_min, w_min))

            E_final_exp = E_search_mean + E_stable_mean

            sweet_print("in day {} we got E_final_mean = {}"
                  .format(d, E_final_exp))

            E_mean_days.append(E_final_exp)

    print("\n ===========================================================")
    print("final values calculation")
    # finally - calculation of control G final values for control and experiment
    E_control = red_far_by_curr(R_control, "control", h_led_control) + \
                white_far_by_curr(W_control, "control", h_led_control)

    sweet_print("E_mean_values_for all days is: \n {}".format(E_mean_days))
    sweet_print("final E_control = {} ppfd, E_exp = {} ppfd".format(
        E_control, np.mean(E_mean_days)))

    G_control_fin = final_intQ(E_control, dM_control, mode="control")
    G_exp_fin = final_intQ(np.mean(E_mean_days), dM_exp, mode="exp")

    sweet_print("G_control_fin  = {} , G_exp_fin = {}".format(G_control_fin, G_exp_fin))

    result = (G_control_fin - G_exp_fin) / G_control_fin * 100
    sweet_print("Global result (Gc - Ge)/Gc * 100% = {} %".format(result))


    con.close()


if __name__ == "__main__":

    search_table = [
        {"number": 1, "red": 130, "white": 130, "finished": 0, 'f': 0, 'q': 0},
        {"number": 2, "red": 70, "white": 190, "finished": 0, 'f': 0, 'q': 0},
        {"number": 3, "red": 190, "white": 70, "finished": 0, 'f': 0, 'q': 0},
        {"number": 4, "red": 40, "white": 160, "finished": 0, 'f': 0, 'q': 0},
        {"number": 5, "red": 160, "white": 40, "finished": 0, 'f': 0, 'q': 0},
        {"number": 6, "red": 100, "white": 100, "finished": 0, 'f': 0, 'q': 0},
        {"number": 7, "red": 220, "white": 220, "finished": 0, 'f': 0, 'q': 0},
        {"number": 8, "red": 25, "white": 235, "finished": 0, 'f': 0, 'q': 0},
        {"number": 9, "red": 145, "white": 115, "finished": 0, 'f': 0, 'q': 0},
        {"number": 10, "red": 85, "white": 55, "finished": 0, 'f': 0, 'q': 0},
        {"number": 11, "red": 205, "white": 175, "finished": 0, 'f': 0, 'q': 0},
        {"number": 12, "red": 55, "white": 85, "finished": 0, 'f': 0, 'q': 0},
        {"number": 13, "red": 175, "white": 205, "finished": 0, 'f': 0, 'q': 0},
        {"number": 14, "red": 115, "white": 145, "finished": 0, 'f': 0, 'q': 0},
        {"number": 15, "red": 235, "white": 25, "finished": 0, 'f': 0, 'q': 0},
        {"number": 16, "red": 17, "white": 138, "finished": 0, 'f': 0, 'q': 0}
    ]

    exp_id = 9

    db = {
        "host": '10.9.0.23',
        "user": 'remote_admin',
        "db": 'experiment',
        "password": "amstraLLa78x[$"
    }

    calculate_final_effect(_db_params=db, _search_table=search_table,
                           _exp_id=exp_id, dM_exp=149, dM_control=232,
                           R_control=10, W_control=250, h_led_control=25, h_led_exp=25)

