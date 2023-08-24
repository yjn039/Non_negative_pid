import math

import numpy as np
import serial
import matplotlib.pyplot as plt
import serial.tools.list_ports
import time
import xlwt
import scipy.signal as signal
import pandas as pd
import sympy
import statsmodels.tsa.api as smt
from scipy.integrate import simpson


from evaluate import np_move_avg,evaluate,osc_period_anaylsis
from pid_core import pid,pid_stability
from collect_data import collect_data_point2_excel
from loss_function import MSELoss,L1Loss,L2Loss


def find_index_first_greater_than_x(lst, x):
    for index, item in enumerate(lst):
        if item > x:
            return index
    return None

def kp_get(exp_val,pid_itr,init_speed,reach_standard):
    kp=0.2
    for i in range(0,100):
      kp=kp+0.4
      time_per_point=collect_data_point2_excel(kp,0,0,pid_itr,exp_val,init_speed)
      file_path = './raw_data1.xls'

      df = pd.read_excel(file_path, sheet_name='Sheet1')
      sensor_data = df[0]

      evaluate_result=evaluate(sensor_data, 10, exp_val, time_per_point, 0, kp, 0, 0)
      reach_time=find_index_first_greater_than_x(sensor_data,exp_val)*time_per_point
      print(reach_time)
      print(kp)
      if i>1:
        if reach_time<reach_standard and evaluate_result[1]!=0:
          print('kp got!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
          break
    #data_result=evaluate(sensor_data, 10, exp_val, 0.1117, 0, kp, 0, 0)
    return(kp)

def ki_get(kp,exp_val,pid_itr,init_speed,eva_standard):
    ki=0
    for i in range(0,100):
      ki=ki+0.01
      time_per_point = collect_data_point2_excel(kp,ki,0,pid_itr,exp_val,init_speed)
      file_path = './raw_data1.xls'
      df = pd.read_excel(file_path, sheet_name='Sheet1')
      sensor_data = df[0]
      evaluate_result=evaluate(sensor_data, 20, exp_val, time_per_point , 0, kp, ki, 0)[3]
      print(evaluate_result)
      print(ki)
      if abs(evaluate_result-exp_val)<eva_standard*exp_val:

          print('ki got!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
          break
    return ki

def kd_get(kp,ki,exp_val,pid_itr,init_speed,std_standard):
    kd=0
    for i in range(0,10):
        kd=kd+0.01
        time_per_point = collect_data_point2_excel(kp, ki, kd, pid_itr, exp_val, init_speed)
        file_path = './raw_data1.xls'
        df = pd.read_excel(file_path, sheet_name='Sheet1')
        sensor_data = df[0]
        evaluate_result = evaluate(sensor_data, 20, exp_val,time_per_point, 0, kp, ki, 0)
        print(evaluate_result)
        print(kd)
        if evaluate_result[2]<std_standard*exp_val  and evaluate_result[2]!=-1:
            print('kd got!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
            break
    evaluate_result= evaluate(sensor_data, 10, exp_val, time_per_point, 1, kp, ki, kd)
    return kd,evaluate_result

def init_pid_para(exp_val,init_kp_itr,init_ki_itr,init_kd_itr,init_speed,init_reachtime_standard,init_ave_standard,init_std_standard):
    kp = kp_get(exp_val, init_kp_itr, init_speed, init_reachtime_standard)
    ki = ki_get(kp, exp_val, init_ki_itr, init_speed, init_ave_standard)
    result_of_init = kd_get(kp, ki, exp_val, init_kd_itr, init_speed, init_std_standard)
    kd=result_of_init[0]
    return kp,ki,kd
