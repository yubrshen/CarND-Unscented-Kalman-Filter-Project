import sys
import os
# from _pickle import dump
sys.path.insert(0, os.path.abspath('..'))

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import math

class AnalyzeResult(object):
    def __init__(self):
        return
    def __load_input_data(self,kalman_input_file):
        print('loading {}.....'.format(kalman_input_file))
        cols = ['type','px_est','py_est','vel_abs','yaw_angle','yaw_rate','vx_est','vy_est','px_meas','py_meas','nis','px_gt','py_gt','vx_gt','vy_gt']
        df = pd.read_csv(kalman_input_file, sep= '\t',names = cols, header=None)
        self.data_label = os.path.basename(kalman_input_file)[:-4]
        self.csv_file_name = os.path.dirname(kalman_input_file) + '/result_analysis/' + self.data_label +'.csv'
        return df
    def disp_nis(self, df):
        # prepare nis data for both Lidar, and Radar
        grouped = df.groupby('type')
        # Two subplots, unpack the axes array immediately
        # f, ax = plt.subplots(1, 2)
        P = {}; P['L'] = {}; P['R'] = {}
        P['L']['low_thres'] = 0.103
        P['L']['high_thres'] = 5.991
        P['L']['title'] = 'lidar nis'

        P['R']['low_thres'] = 0.352
        P['R']['high_thres'] = 7.815
        P['R']['title'] = 'radar nis'

        for (key, item) in grouped:
            plt.plot(item.index.values, item['nis'])
            plt.plot([item.index.values[0], item.index.values[-1]], np.full(2, P[key]['low_thres']))
            plt.plot([item.index.values[0], item.index.values[-1]], np.full(2, P[key]['high_thres']))
            plt.title(P[key]['title'] + " " + self.data_label)
            plt.show()
        return
    def visulize_position(self,df):
        gt = plt.plot(df['px_gt'], df['py_gt'],label='Ground Truth Position')
        est = plt.plot(df['px_est'], df['py_est'],label='Estimated Position')
        meas = plt.scatter(df['px_meas'], df['py_meas'],label='measurement')
        plt.xlabel('Postiion estimation' + " " + self.data_label)
        plt.ylabel('py')
        plt.title('px')
        plt.legend(loc='upper right')
        plt.show()
        return
    def visualize_velocity(self,df):
        v_gt = np.sqrt((df['vx_gt'].values **2 + df['vy_gt'].values **2))
#         v_est = np.sqrt((df['vx_est'].values **2 + df['vy_est'].values **2))
        # v_est = abs(df['vel_abs'].values)  # force the velocity to be positive
        v_est = (df['vel_abs'].values)
        plt.plot(v_gt, label='Ground Truth Velocity')
        plt.plot(v_est, label='Estimated Velocity')
        plt.legend(loc='upper right')
        plt.show()
        return
    def visualize_yaw_angle(self,df):
        gt = np.arctan2(df['vy_gt'].values,  df['vx_gt'].values)
        est = df['yaw_angle'].values
        #est = np.arctan2(np.sin(df['yaw_angle'].values), np.cos(df['yaw_angle'].values))
        plt.plot(gt, label='Ground Truth Yaw Angle')
        plt.plot(est, label='Estimated Yaw Angle')
        plt.plot(abs(gt-est), label="Diff abs GT and Est", color='b')
        plt.legend(loc='upper right')
        plt.show()
        return
    def run_data(self, number):
        number_str = str(number)
        print('####data sample ' + number_str + ' ###')
        kalman_input_file = r'../data/sample-laser-radar-measurement-data-' + number_str + '_output.txt'
        df = self.__load_input_data(kalman_input_file)
        self.disp_nis(df)
        self.visulize_position(df)
        self.visualize_velocity(df)
        self.visualize_yaw_angle(df)
        df.to_csv(self.csv_file_name)
        print(self.csv_file_name + ' saved')
        return df
    def run(self):
        self.run_data(1)
        self.run_data(2)
        # plt.show()
        return
if __name__ == "__main__":
    obj= AnalyzeResult()
    obj.run()
