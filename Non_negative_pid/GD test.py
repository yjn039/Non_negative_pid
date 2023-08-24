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

file_path='./raw_data.xls'
config_number=1
dataset_number=1
sheet_name_list=['200group','300group','400group','500group']
dataset_list=['data1','data2','data3','data4','data5','data6']
df= pd.read_excel(file_path,sheet_name=sheet_name_list[config_number-1])
sensor_data=df[dataset_list[dataset_number-1]]


def MSELoss(x:list,y:list):
    assert len(x)==len(y)
    x=np.array(x)
    y=np.array(y)
    loss=np.sum(np.square(x-y)/len(x))
    return loss

def L2Loss(x:list,y:list):
    assert len(x)==len(y)
    x=np.array(x)
    y=np.array(y)
    loss=np.sqrt(np.sum(np.square(x-y)/len(x)))
    return loss

def L1Loss(x:list,y:list):
    assert len(x)==len(y)
    x=np.array(x)
    y=np.array(y)
    loss=np.sum(np.abs(x-y)/len(x))
    return loss


pid_itr=len(sensor_data)
exp_val_list=[200,300,400,500]
exp_val=exp_val_list[config_number-1]
time_per_point=0.1117
window_length=20
time_thres=1
standard=[exp_val,0,0.0,exp_val]


#plt.figure()
#plt.plot(sensor_data)

evaluate_result=[287.2, 10.740895539522171, 4.2368698630968336, 199.35498281786943]
#error
error=[0]*4


mseloss=MSELoss(standard,evaluate_result)
l1loss=L1Loss(standard,evaluate_result)
l2loss=L2Loss(standard,evaluate_result)

print(evaluate_result)
print('mseloss')
print(mseloss)
print('l1loss')
print(l1loss)
print('l2loss')
print(l2loss)


