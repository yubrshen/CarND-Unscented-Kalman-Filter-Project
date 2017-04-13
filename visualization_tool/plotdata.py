import sys
import os
# from _pickle import dump
sys.path.insert(0, os.path.abspath('..'))

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import math


class PlotData(object):
    def __init__(self):
       
        return
    def __load_input_data(self,kalman_input_file):
        
        print('processing {}.....'.format(kalman_input_file))
        self.previous_timestamp = None   
        
        entries = []
        cols = ['type_meas', 'px_meas','py_meas','vx_meas','vy_meas','timestampe','delta_time','px_gt','py_gt','vx_gt','vy_gt',
                'rho_meas','phi_meas','rho_dot_meas', 'rho_gt','phi_gt','rho_dot_gt','vel_abs','yaw_angle']
        with open(kalman_input_file, "r") as ins:
            for line in ins:
                entries.append(self.__process_line(line))
        df = pd.DataFrame(entries, columns=cols)
        self.csv_file_name = os.path.dirname(kalman_input_file) + '/preprocess/' + os.path.basename(kalman_input_file)[:-4] +'.csv'
        return df
    def __cal_input_rmse(self,df):
        lidar_df = df[df['type_meas'] == 'L']
        radar_df = df[df['type_meas'] == 'R']
        print('lidar only: {}'.format(self.__cal_rmse(lidar_df)))
        print('radar only: {}'.format(self.__cal_rmse(radar_df)))
        print('all data: {}'.format(self.__cal_rmse(df)))
        


        df = self.__add_first_derivative(df)
        df = self.__add_second_derivative(df)
        
        #         print("lidar measurement noise {}, {}".format(np.var(lidar_df['px_meas'] - lidar_df['px_gt']), np.var(lidar_df['py_meas'] - lidar_df['py_gt'])))
#         print("radar measurement noise {}, {}, {}".format(np.var(radar_df['rho_meas'] - radar_df['rho_gt']), np.var(radar_df['phi_meas'] - radar_df['phi_gt']),
#                                                  np.var(radar_df['rho_dot_meas'] - radar_df['rho_dot_gt'])))
        print("motion noise {}, {}".format(np.std(df['vel_acc']), np.std(df['yaw_acc'])))
        print("velocity mean {}, yaw rate mean{}".format(np.mean(df['vel_abs']), np.mean(np.absolute(df['yaw_angle']))))
        
        subdf = df[['yaw_diff', 'yaw_rate', 'yaw_acc','vel_abs', 'vel_acc' ]]
        print('ground truth statistics')
        print(subdf.describe())
        
        return df
    def __add_second_derivative(self, df):
        yaw_acc_vec = []
        for i in range(df.shape[0]):
            if i == 0:
                yaw_acc_vec.append(0)
                continue
            delta_time = df.iloc[i]['delta_time']
            if  delta_time== 0:
                yaw_acc_vec.append(yaw_acc_vec[-1])
                continue
            yaw_acc = (df.iloc[i]['yaw_rate'] - df.iloc[i-1]['yaw_rate'])/delta_time
            yaw_acc_vec.append(yaw_acc)
            
        df['yaw_acc'] = yaw_acc_vec
                           
        return df
    def __add_first_derivative(self, df):
        vel_acc_vec = []
        yaw_rate_vec = []
        yaw_diff_vec = []
        for i in range(df.shape[0]):
            if i == 0:
                vel_acc_vec.append(0)
                yaw_diff_vec.append(0)
                yaw_rate_vec.append(0)
                
                continue
            delta_time = df.iloc[i]['delta_time']
            if  delta_time== 0:
                vel_acc_vec.append(vel_acc_vec[-1])
                yaw_diff_vec.append(yaw_diff_vec[-1])
                yaw_rate_vec.append(yaw_rate_vec[-1])
                
                continue
            #vel acceleration
            vel_acc = (df.iloc[i]['vel_abs'] - df.iloc[i-1]['vel_abs'] )/delta_time
            vel_acc_vec.append(vel_acc)
            #yaw rate
            yaw_diff = 0
            if (df.iloc[i-1]['yaw_angle'] != 0) and ((df.iloc[i]['yaw_angle'] != 0)):
                yaw_diff = (df.iloc[i]['yaw_angle'] - df.iloc[i-1]['yaw_angle'])
                yaw_diff = self.get_normalziaed_angle(yaw_diff)
                
            
            yaw_diff_vec.append(yaw_diff)
            yaw_rate_vec.append(yaw_diff/delta_time)
        df['yaw_diff'] = yaw_diff_vec
        df['yaw_rate'] = yaw_rate_vec
        df['vel_acc'] = vel_acc_vec
        
        
        return df
    def get_normalziaed_angle(self, angle):
        while(angle> math.pi):
            angle -=  2*math.pi
        while(angle < -math.pi):
            angle +=  2*math.pi
        return angle
    def __cal_rmse(self, df):
        px_rmse = math.sqrt(((df['px_meas'] - df['px_gt']).values ** 2).mean())
        py_rmse = math.sqrt(((df['py_meas'] - df['py_gt']).values ** 2).mean())
        
        sub_df = df[(df['vx_meas'] !=0) & (df['vy_meas'] !=0) ]
        
        if sub_df.shape[0] !=0:
            vx_rmse = math.sqrt(((sub_df['vx_meas'] - sub_df['vx_gt']).values ** 2).mean())
            vy_rmse = math.sqrt(((sub_df['vy_meas'] - sub_df['vy_gt']).values ** 2).mean())
        else:
            vx_rmse = 0
            vy_rmse = 0
    
        
        return px_rmse,py_rmse,vx_rmse,vy_rmse
    def __process_line(self, line):
        px_meas = 0
        py_meas = 0
        
        rho_meas = 0
        phi_meas = 0
        rho_dot_meas = 0
        
        vx_meas = 0
        vy_meas = 0
       
        timestampe = 0
        px_gt = 0
        py_gt = 0
        vx_gt = 0
        vy_gt = 0
        
        rho_gt =0
        phi_gt = 0
        rho_dot_gt = 0
        
        
        
        
        if 'L'in line:
            type_meas,px_meas,py_meas, timestampe,px_gt,py_gt,vx_gt,vy_gt=line[:-1].split("\t")
            px_meas = float(px_meas)
            py_meas = float(py_meas)
            timestampe = int(timestampe)
            px_gt = float(px_gt)
            py_gt = float(py_gt)
            vx_gt = float(vx_gt)
            vy_gt = float(vy_gt)
        elif 'R' in line:
            type_meas,rho_meas,phi_meas,rho_dot_meas, timestampe,px_gt,py_gt,vx_gt,vy_gt=line[:-1].split("\t")
            rho_meas = float(rho_meas)
            phi_meas = float(phi_meas)
            rho_dot_meas = float(rho_dot_meas)
            timestampe = int(timestampe)
            px_gt = float(px_gt)
            py_gt = float(py_gt)
            vx_gt = float(vx_gt)
            vy_gt = float(vy_gt)
            
            px_meas = rho_meas * math.cos(phi_meas)
            py_meas = rho_meas * math.sin(phi_meas)
            
            vx_meas = rho_dot_meas * math.cos(phi_meas)
            vy_meas = rho_dot_meas * math.sin(phi_meas)
            
            
            rho_gt = math.sqrt(px_gt ** 2 + py_gt ** 2)
            if px_gt != 0:
                phi_gt = math.atan(py_gt/px_gt)
            else:
                print('px_gt is zero')
            
            if rho_gt != 0:
                rho_dot_gt = (px_gt * vx_gt + py_gt * vy_gt) / rho_gt
            else:
                print('rho_gt is zero')
        else:
            raise("unexpected line" + line)
        
        delta_time = 0
        
        if self.previous_timestamp is not None:
            delta_time = (timestampe - self.previous_timestamp)/1000000.0
        
        self.previous_timestamp = timestampe
        
        vel_abs = 0
        yaw_angle = 0
        
        vel_abs = math.sqrt(vx_gt*vx_gt+ vy_gt*vy_gt)   
      
        yaw_angle = math.atan2(vy_gt, vx_gt)
        
        
        return type_meas, px_meas,py_meas,vx_meas,vy_meas,timestampe,delta_time, px_gt,py_gt,vx_gt,vy_gt,\
            rho_meas,phi_meas,rho_dot_meas, rho_gt,phi_gt,rho_dot_gt,vel_abs,  yaw_angle
        
    
   
    def disp_input(self,df):
        colors = ['b', 'c', 'y', 'm', 'r']
        gt = plt.plot(df['px_gt'], df['py_gt'],marker='*', color= colors[0])
        meas = plt.scatter(df['px_meas'], df['py_meas'],marker='v',color=colors[4])
        plt.legend((gt, meas),
           ('ground truth', 'measurement'))
        plt.show()
        return
    def run_data_1(self):
        print('####data sample 1###')
        kalman_input_file = r'../data/sample-laser-radar-measurement-data-1.txt'
        df = self.__load_input_data(kalman_input_file)
        df = self.__cal_input_rmse(df)
        
        df.to_csv(self.csv_file_name)
        print(self.csv_file_name + ' saved')
        self.disp_input(df)
        return df
    
    def run_data_2(self):
        print('####data sample 2###')
        kalman_input_file = r'../data/sample-laser-radar-measurement-data-2.txt'
        df = self.__load_input_data(kalman_input_file)
        df = self.__cal_input_rmse(df)
        
        df.to_csv(self.csv_file_name)
        print(self.csv_file_name + ' saved')
        self.disp_input(df)
        return df
    
   
        
    def run(self):
#         self.run_data_1()
        self.run_data_2()
    

        return



if __name__ == "__main__":   
    obj= PlotData()
    obj.run()