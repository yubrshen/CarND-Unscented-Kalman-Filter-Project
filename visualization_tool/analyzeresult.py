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
        self.csv_file_name = os.path.dirname(kalman_input_file) + '/result_analysis/' + os.path.basename(kalman_input_file)[:-4] +'.csv'
      
        return df
    
    def disp_nis(self, df):
        
        
        # Two subplots, unpack the axes array immediately
        f, ax1 = plt.subplots(1, 1)
        
        if df['type'][0] == 'L':
            low_thres = 0.103
            high_thres = 5.991
            title = 'lidar nis'
        else:
            low_thres = 0.352
            high_thres = 7.815 
            title = 'radar nis'
        
        sub_df = df
       

        ax1.plot(sub_df.index.values, sub_df['nis'])
        ax1.plot([sub_df.index.values[0], sub_df.index.values[-1]], np.full(2, low_thres))
        ax1.plot([sub_df.index.values[0], sub_df.index.values[-1]], np.full(2, high_thres))
        ax1.set_title(title)
        
      
        plt.show()
        return
    
    
    def visulize_position(self,df):
        gt = plt.plot(df['px_gt'], df['py_gt'],label='Ground Truth Position')
        est = plt.plot(df['px_est'], df['py_est'],label='Estimated Position')
        meas = plt.scatter(df['px_meas'], df['py_meas'],label='measurement')

        plt.xlabel('Postiion estimation')
        plt.ylabel('py')
        plt.title('px')
        plt.legend(loc='upper right')
        
        return
    
    
    def visualize_velocity(self,df):
        v_gt = np.sqrt((df['vx_gt'].values **2 + df['vy_gt'].values **2))
#         v_est = np.sqrt((df['vx_est'].values **2 + df['vy_est'].values **2))
        v_est = df['vel_abs'].values
        
        
        
        plt.plot(v_gt, label='Ground Truth Velocity')
        plt.plot(v_est, label='Estimated Velocity')
        plt.legend(loc='upper right')
        return
    
    def visualize_yaw_angle(self,df):
        gt = np.arctan2(df['vy_gt'].values,  df['vx_gt'].values)

        est = df['yaw_angle'].values
        
        plt.plot(gt, label='Ground Truth Yaw Angle')
        plt.plot(est, label='Estimated Yaw Angle')
        plt.legend(loc='upper right')
        return
    
    def run_data_1(self):
        print('####data sample 1###')
        kalman_input_file = r'../data/sample-laser-radar-measurement-data-1_output.txt'
        df = self.__load_input_data(kalman_input_file)
        self.disp_nis(df)

        return df
    def run_data_2(self):
        print('####data sample 2###')
        kalman_input_file = r'../data/sample-laser-radar-measurement-data-2_output.txt'
        df = self.__load_input_data(kalman_input_file)
#         self.visulize_position(df)
#         self.visualize_velocity(df)
        self.visualize_yaw_angle(df)
        df.to_csv(self.csv_file_name)
        print(self.csv_file_name + ' saved')
        return df
    
   
        
    def run(self):
#         self.run_data_1()
        self.run_data_2()
        plt.show()
        
#         self.visulize_data(df)

        return



if __name__ == "__main__":   
    obj= AnalyzeResult()
    obj.run()