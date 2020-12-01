#!/usr/bin/env python
# -*- coding: utf-8 -*-


import numpy as np
import dash
import dash_core_components as dcc
import dash_html_components as html
from dash.dependencies import Input, Output
import plotly.express as px
import time
import datetime
import os
import rospy
from std_msgs.msg import String
from custom_logger import CustomLogger
import pymysql


class MYSQLHandler(object):

    def __init__(self):
        print("========== start init")




        # start node
        rospy.init_node('mysql_data_plotter_server', log_level=rospy.DEBUG, anonymous=True)

        # self logging
        self._logname = rospy.get_param('~mysql_data_plotter_log_name', 'mysql_data_plotter')
        self._log_node_name = rospy.get_param('~mysql_data_plotter_log_node_name', 'mysql_data_plotter_log_node')

        # create log topic publisher
        self._log_pub = rospy.Publisher(self._log_node_name, String, queue_size=10)

        # logger
        self._logger = CustomLogger(name=self._logname, logpub=self._log_pub)
        self._logger.debug("mysql_data_plotter_server init")

        # for default we will use current ip in tun0 protocol
        # if there is no any tun0 - it must be a critical error

        self.ipv4 = os.popen('ip addr show tun0').read().split("inet ")[1].split("/")[0]

        self.web_addr = self.ipv4
        self.web_port = 8090  # on every rpi
        self.list_of_hdf = list()
        self.metastring = "experiment metadata: \n"

        # TODO load description from experiments table and add to metastring
        # self.list_of_datasets =

        print("========== start get list of datasets")


        # TODO; fix -all below only for test  ||||
        #                            vvvv
        self.list_of_tables = rospy.get_param('~mysql_data_plotter_raw_topics')
        # self.list_of_tables = [  # TODO must be loaded from .launch
        #     {'name': '/bmp180_1_temp_pub', 'id': 1, 'type': 'temperature', 'dtype': 'float64', 'units': 'C', 'status': 'raw_data'},
        #     {'name': '/bmp180_1_pressure_pub', 'id': 2, 'type': 'pressure', 'dtype': 'float64', 'units': 'kPa', 'status': 'raw_data'},
        #     {'name': '/raw_co2_pub', 'type': 'co2', 'id': 3, 'dtype': 'float64', 'units': 'ppmv', 'status': 'raw_data'},
        #     {'name': '/si7021_1_hum_pub', 'id': 4, 'type': 'humidity', 'dtype': 'float64', 'units': 'percents', 'status': 'raw_data'},
        #     {'name': '/si7021_1_temp_pub', 'id': 5, 'type': 'temperature', 'dtype': 'float64', 'units': 'C', 'status': 'raw_data'}
        # ]

        print("========== end of init")


    def get_dataset(self, name):
        print("========== start get dataset {}".format(name))

        info_dict = None

        # lets find dict with same name as user wants to see
        for i in self.list_of_tables:
            if i['name'] == name:
                info_dict = i

        # dirty
        meta = info_dict

        # table name, where data located must be placed in field "status"
        if info_dict['status'] == 'raw_data':
            # if raw, it must be in raw_data table and must be defined by sensor_id
            sensor_id = info_dict['id']

            con = pymysql.connect(host='localhost',
                                  user='admin',
                                  password='admin',
                                  # db='experiment',
                                  charset='utf8mb4',
                                  cursorclass=pymysql.cursors.DictCursor)

            cur = con.cursor()

            cur.execute("use experiment")

            comm_str = 'select data, time from raw_data where sensor_id = {}'.format(sensor_id)


            self._logger.debug("comm_str: {}".format(comm_str))



            cur.execute(comm_str)

            res_list = cur.fetchall()  # it must be array of dicts
            # where each dict is one line from db

            con.close()


            print("========== end of get dataset")
            return res_list, meta

    # def show_plot(self, np_arr, meta):


print("========== start all")

external_stylesheets = ['https://codepen.io/chriddyp/pen/bWLwgP.css']

app = dash.Dash(__name__, external_stylesheets=external_stylesheets)

hh = MYSQLHandler()
# hh.get_list_of_datasets()
# numpy_data, meta = hh.get_dataset('Raw_Data/si7021_1_hum_pub')

available_indicators = list(x['name'] for x in  hh.list_of_tables)
print("available_indicators: {}".format(available_indicators))
print("========== hh created")

app.layout = html.Div([
    html.H1(children='MySQL Test'),

    html.Div(children=hh.metastring),
    html.Div([
        # html.Div(id='metastr', children=str(meta)),
        dcc.Graph(id='graph'),
        dcc.Dropdown(
            id='yaxis-column',
            options=[{'label': i, 'value': i} for i in available_indicators],
            value='/si7021_1_hum_pub'  # TODO for test only
        )
        ])])

print("========== app created")


@app.callback(
    Output('graph', 'figure'),
    [
    # [Input('xaxis-column', 'value'),
     Input('yaxis-column', 'value')
     # Input('xaxis-type', 'value'),
     # Input('yaxis-type', 'value'),
     # Input('year--slider', 'value')
    ])
# def update_graph(xaxis_column_name, yaxis_column_name,
#                  xaxis_type, yaxis_type,
#                  year_value):
def update_graph(yaxis_column_name):
    print("========== start update_graph callback")
    full_data_list, meta = hh.get_dataset(yaxis_column_name)

    # print("res is: {}".format(full_data_list))
    print(meta)
    # convert timestamps to datetime str
    # times = list()
    # for t in numpy_data[0, :]:
    #     times.append(datetime.datetime.fromtimestamp(t).strftime('%Y_%m_%d,%H:%M:%S'))
    datas_list = list(x['data'] for x in full_data_list)
    times_list = list(x['time'] for x in full_data_list)

    print("========== times converted")

    # dff = pd.(numpy_data)

    if meta['status'] == 'raw_data':
        fig = px.line(x=times_list,
                         y=datas_list)
                         # hover_name=dff[dff['Indicator Name'] == yaxis_column_name]['Country Name'])
        print("========== fig created")
        fig.update_layout(margin={'l': 40, 'b': 40, 't': 10, 'r': 0}, hovermode='closest')
        # fig.update_xaxes(title=xaxis_column_name,
        #                  type='linear' if xaxis_type == 'Linear' else 'log')
        print("========== fig layout updated")
        fig.update_yaxes(title=yaxis_column_name, type='linear')
                         # type='linear' if yaxis_type == 'Linear' else 'log')
        return fig





if __name__ == "__main__":
    # app.run_server(debug=True, host='192.168.100.9', port=8090)
    app.run_server(debug=True, host=hh.web_addr, port=hh.web_port)




    # hh.show_plot(numpy_data, meta)

