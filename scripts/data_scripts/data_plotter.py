import h5py
import numpy as np
import dash
import dash_core_components as dcc
import dash_html_components as html
from dash.dependencies import Input, Output
import plotly.express as px
import pandas as pd
import time
import datetime
import os

class hdf_handler(object):

    def __init__(self):
        print("========== start init")
        # self.data_path = "/home/greencrow/raw_data.hdf5"
        self.data_path = "/home/pi/test_exp_data/raw_data.hdf5"

        # for default we will use current ip in tun0 protocol
        # if there is no any tun0 - it must be a critical error

        self.ipv4 = os.popen('ip addr show tun0').read().split("inet ")[1].split("/")[0]

        self.web_addr = self.ipv4
        self.web_port = "8090"  # on every rpi
        self.list_of_hdf = list()
        self.metastring = "experiment metadata: \n"
        # self.list_of_datasets =
        with h5py.File(self.data_path, 'r') as f:
            self.global_meta = dict.fromkeys(f.attrs.keys())
            for m in f.attrs.keys():
                self.global_meta[m] = f.attrs[m]
                self.metastring+='{}: {} \n'.format(m, f.attrs[m])
                print('{}: {}'.format(m, f.attrs[m]))
        print("========== end of init")

    def get_list_of_datasets(self):
        print("========== start get list of datasets")
        list_of_hdf = list()

        def get_all(name):
            list_of_hdf.append(name)
            print(name)

        def _get_datasets(name, obj):
            if (type(obj) == h5py._hl.dataset.Dataset):
                # list_of_hdf.append((name, obj))
                list_of_hdf.append(name)
            # self.list_of_hdf = list_of_hdf
            print(name)

        with h5py.File(self.data_path, 'r') as f:

            f.visititems(_get_datasets)
            # f.visit(get_all)
            self.list_of_hdf = list_of_hdf
            print(self.list_of_hdf)

        print("========== end of get list of datasets")

    def get_dataset(self, name):
        print("========== start get dataset {}".format(name))
        with h5py.File(self.data_path, 'r') as f:
            hdf_data = f[name]
            print(np.shape(hdf_data))
            numpy_data = f[name][:, 0:np.shape(hdf_data)[1]-1]

            # arr [points from 1st axe, points from 2 axe, ...]
            print(numpy_data[:, 0:10])

            meta = dict.fromkeys(hdf_data.attrs.keys())
            for m in hdf_data.attrs.keys():
                meta[m] = hdf_data.attrs[m]
                print('{}: {}'.format(m, hdf_data.attrs[m]))

            print(type(hdf_data))
            print(type(numpy_data))
            print(type(meta))
            print(meta)

            print("========== end of get dataset")
            return numpy_data, meta

    # def show_plot(self, np_arr, meta):


print("========== start all")

external_stylesheets = ['https://codepen.io/chriddyp/pen/bWLwgP.css']

app = dash.Dash(__name__, external_stylesheets=external_stylesheets)

hh = hdf_handler()
hh.get_list_of_datasets()
numpy_data, meta = hh.get_dataset('Raw_Data/si7021_1_hum_pub')

available_indicators = hh.list_of_hdf
print("available_indicators: {}".format(available_indicators))
print("========== hh created")

app.layout = html.Div([
    html.H1(children='Hdf5 Test'),

    html.Div(children=hh.metastring),
    html.Div([
        # html.Div(id='metastr', children=str(meta)),
        dcc.Graph(id='graph'),
        dcc.Dropdown(
            id='yaxis-column',
            options=[{'label': i, 'value': i} for i in available_indicators],
            value='Raw_Data/bmp180_1_temp_pub'
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
    numpy_data, meta = hh.get_dataset(yaxis_column_name)
    # convert timestamps to datetime str
    times = list()
    for t in numpy_data[0, :]:
        times.append(datetime.datetime.fromtimestamp(t).strftime('%Y_%m_%d,%H:%M:%S'))

    print("========== times converted")

    # dff = pd.(numpy_data)

    fig = px.line(x=times,
                     y=numpy_data[1, :])
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
    app.run_server(debug=True, host='10.9.0.13', port=8090)




    # hh.show_plot(numpy_data, meta)

