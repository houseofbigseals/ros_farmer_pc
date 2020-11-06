
import numpy as np
import pylab as pl
import pymysql
from scipy.optimize import curve_fit
from exp_units_conversions import red_far_by_curr, white_far_by_curr, dry_intQ


def load_point(_db_params, point_id, cut_num = 200, show=True):
    # get one point remotely and calculate F
    con = pymysql.connect(host=_db_params["host"],
                          user=_db_params["user"],
                          password=_db_params["password"],
                          # db='experiment',
                          charset='utf8mb4',
                          cursorclass=pymysql.cursors.DictCursor)

    cur = con.cursor()

    cur.execute("use experiment")


    comm_str = "select step_id, red, white, start_time, end_time from exp_data where point_id={}".format(
        point_id
    )

    resp = cur.execute(comm_str)

    rows = cur.fetchall()
    # print(rows)
    t_start = rows[0]['start_time']
    t_stop = rows[0]['end_time']
    red = rows[0]['red']
    white = rows[0]['white']

    # for now we will handle one point differentiation in this callback
    # select time, data from raw_data where sensor_id = 3 and time
    comm_str = "select time, data from raw_data where exp_id = {} and sensor_id = {} " \
               "and time > '{}' and time < '{}'".format(
        2, 3, t_start, t_stop)

    # print("comm_str: {}".format(comm_str))  # TODO: remove after debug


    resp = cur.execute(comm_str)

    rows = cur.fetchall()

    # then lets find which rows correspond to search_table
    # for db_row in rows:

    print(len(rows))

    # self._logger.debug(len(resp))
    co2_array = [x['data'] for x in rows]
    time_array = [x['time'] for x in rows]

    con.close()
    converted_time = [(t - time_array[0]).total_seconds() for t in time_array]

    # cut ~ first 200 points

    cut_time = converted_time[cut_num:]
    cut_co2 = co2_array[cut_num:]
    cut_converted_time = [t - cut_time[0] for t in cut_time]
    print(len(cut_time))
    print(len(cut_co2))

    if show:
        # 2D plot
        fig = pl.figure()
        pl.plot(cut_converted_time, cut_co2, 'ob', label="CO2, ppm")

        pl.legend()
        pl.grid()
        pl.show()

    return cut_converted_time, cut_co2, red, white

def exp_approximation(co2, times,red, white, show=True):
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
    print('fit exp: a={:.4f}, b={:.6f}'.format(epopt[0], epopt[1]))
    print('fit lin: a={:.4f}, b={:.6f}'.format(lpopt[0], lpopt[1]))
    y_eopt = exp_func(x, *epopt)
    y_lopt = lin_func(x, *lpopt)

    # point for derivative nov is in middle of cutted time interval
    t_derivative = int(len(x)/2)

    F_lin = lpopt[0]
    F_exp = exp_deriv(t_derivative, *epopt)
    print("F_lin = {},  F_exp = {}".format(F_lin, F_exp))

    # dC - first derivative of co2 concentration in ppnmv/sec
    # E - light intencity im mkmoles/m2*sec
    # dT - time period of measure im sec

    dC = -1 * F_exp
    E = white_far_by_curr(white) + red_far_by_curr(red)
    print ("Ired = {}, Iwhite = {}, dC = {},  E = {}".format(red, white, dC, E))
    # dT = (time_array[len(time_array) - 1] - time_array[0]).total_seconds()
    dT = 900.0  # full time of one search step

    dry_q = dry_intQ(dC, E, dT)

    print("dry_q = {}".format(dry_q))

    # 2D plot
    if show:
        fig = pl.figure()


        # t = range(len(raw_co2))
        # pl.xticks(t, times, rotation='vertical')
        # pl.plot(t, raw_co2, '-.g', label="CO2, ppm")
        # pl.plot(t[number_of_cut::], cut_co2, '-b', label="cut CO2, ppm")
        pl.plot(x, y, '-.b', label="cut CO2, ppm")
        pl.plot(x, y_eopt, '-g', label="exp appr CO2, ppm")
        pl.plot(x, y_lopt, '-r', label="lin appr CO2, ppm")

        # pl.plot(t, fr_fw*200, '-b', label="FARred/FARwhite")
        # pl.plot(t, far, '-r', label="FAR summ, mkmoles")
        # pl.plot(t,  air*400, '-k', label="Airflow ON")
        # pl.plot(t, co2K30, '-c', label="CO2 outside")
        # pl.ylabel('CO2, ppm')
        pl.xlabel('time')
        # pl.title('fit: a={:.4f}, b={:.6f}'.format(epopt[0], epopt[1]))
        pl.legend()
        pl.grid()
        pl.show()
    return F_lin, F_exp, dry_q

if __name__ == "__main__":
    db = {
        "host": '10.9.0.23',
        "user": 'remote_admin',
        "db": 'experiment',
        "password": "amstraLLa78x[$"
    }

    x, y, r, w = load_point(db, 119, cut_num=200, show=True)
    fl, fe, q = exp_approximation(y, x, r, w, show=True)



    # l = [25, 50, 75, 100, 125]
    # cut_num = 100
    # res100 = list()
    # for i in l:
    #     x, y, r, w = load_point(db, i, cut_num=cut_num,show=False)
    #     fl, fe = exp_approximation(y, x, r, w, show=False)
    #     print("cut = {} RESULTS point {} fl = {}, fe = {}".format(cut_num, i, fl, fe))
    #     res100.append({'point':i, 'cut_num':cut_num, 'FLin':fl, 'FExp':fe})

    # cut_num = 150
    # res150 = list()
    # for i in l:
    #     x, y = load_point(db, i, cut_num=cut_num, show=False)
    #     fl, fe = exp_approximation(y, x, show=False)
    #     print("cut = {} RESULTS point {} fl = {}, fe = {}".format(cut_num, i, fl, fe))
    #     res150.append({'point':i, 'cut_num':cut_num, 'FLin':fl, 'FExp':fe})
    #
    # cut_num = 200
    # res200 = list()
    # for i in l:
    #     x, y = load_point(db, i, cut_num=cut_num, show=False)
    #     fl, fe = exp_approximation(y, x, show=False)
    #     print("cut = {} RESULTS point {} fl = {}, fe = {}".format(cut_num, i, fl, fe))
    #     res200.append({'point':i, 'cut_num':cut_num, 'FLin':fl, 'FExp':fe})
    #
    #
    # print("===================================================")
    # print(res100)
    # print("===================================================")
    # print(res150)
    # print("===================================================")
    # print(res200)
    # for i in range(20, 50):
    #     x, y = load_point(db, i, show=False)
    #     print("RESULTS:", exp_approximation(y, x, show=False))