import plotly.offline as py
from plotly.graph_objs import *
import pandas as pd
import math

# py.init_notebook_mode()
kalman_output_file = r'../data/sample-laser-radar-measurement-data-1_output.txt'

my_cols=['px_est','py_est','vel_abs','yaw_angle','yaw_rate','vx_est','vy_est','px_meas','py_meas','nis','px_gt','py_gt','vx_gt','vy_gt']
with open(kalman_output_file) as f:
    table_ekf_output = pd.read_table(f, sep='\t', header=None, names=my_cols, lineterminator='\n')
    




#estimations
trace1 = Scatter(
    x=table_ekf_output['px_est'],
    y=table_ekf_output['py_est'],
    xaxis='x2',
    yaxis='y2',
    name='KF- Estimate'
)

#Measurements
trace2 = Scatter(
    x=table_ekf_output['px_meas'],
    y=table_ekf_output['py_meas'],
    xaxis='x2',
    yaxis='y2',
    name = 'Measurements',
    mode = 'markers'
)

#Measurements
trace3 = Scatter(
    x=table_ekf_output['px_gt'],
    y=table_ekf_output['py_gt'],
    xaxis='x2',
    yaxis='y2',
    name = 'Ground Truth'
)

data = [trace1, trace2, trace3]
# data = [trace3]

layout = Layout(
    xaxis2=dict(
   
        anchor='x2',
        title='px'
    ),
    yaxis2=dict(
    
        anchor='y2',
        title='py'
    )
)

fig = Figure(data=data, layout=layout)
py.plot(fig, filename= 'EKF.html')

