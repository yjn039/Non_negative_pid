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
from kp_ki_kd import kp_get,ki_get,kd_get


#Init pid_para
init_kp_itr=100
init_ki_itr=300
init_kd_itr=400



delta_kp=0.1
delta_ki=0.01
delta_kd=0.01
pid_itr=300
exp_val=500   #ul/min
itr_num=10
init_speed=0
window_length=20
goal=[exp_val,0,0,exp_val]
stop_condition=[1.1*exp_val,3.0,0.02*exp_val,0.995*exp_val]
init_reachtime_standard=stop_condition[1]*1/2
init_ave_standard=0.005
init_std_standard=0.02
weight=[0.05,1,1,1]
pid_change_list=[[0.1,0,0],[-0.1,0,0],[0,0.01,0],[0,-0.01,0],[0,0,0.01],[0,0,-0.01]]





def collect_and_evaluate_data(kp,ki,kd,pid_itr,exp_val,init_speed,window_length,graph):
    global time_per_point
    time_per_point = collect_data_point2_excel(kp, ki, kd, pid_itr, exp_val, init_speed)

    file_path = './raw_data1.xls'

    df = pd.read_excel(file_path, sheet_name='Sheet1')
    sensor_data = df[0]

    evaluate_result = evaluate(sensor_data, window_length, exp_val, time_per_point,graph,kp,ki,kd)

    return evaluate_result




def try_nearest_parameter(goal,pid_change_list,kp,ki,kd,pid_itr,exp_val,init_speed,window_length,now_loss):
    last_loss=now_loss
    loss=[1000000]*6
    pid_para=[kp,ki,kd]
    for i in range(0,6):
        pid_para_result=[j + k for j, k in zip(pid_para, pid_change_list[i])]
        for j in range (0,3):
            if pid_para_result[j]<0:
                pid_para_result[j]=0
        if pid_para_result!=pid_para :
           evaluate_result=collect_and_evaluate_data(pid_para_result[0],pid_para_result[1],pid_para_result[2],pid_itr,exp_val,init_speed,window_length,0)
           loss[i]=L1Loss(evaluate_result,goal,weight)
           print(loss[i])
           print(pid_para_result)
        else:
            loss[i]=last_loss
    min_loss=np.argmin(np.array(loss))
    now_loss=np.min(loss)
    print('loss_function' + str(loss))
    if now_loss<last_loss:
        # print(loss)
        pid_para_result = [j + k for j, k in zip(pid_para, pid_change_list[min_loss])]
        for j in range(0, 3):
            if pid_para_result[j] < 0:
                pid_para_result[j] = 0
        #print('loss_function' + str(loss))
        print('pid_para_choosed')
        print(pid_para_result)
        evaluate_result = collect_and_evaluate_data(pid_para_result[0], pid_para_result[1], pid_para_result[2], pid_itr,
                                                    exp_val, init_speed, window_length, 1)
        kp = pid_para_result[0]
        ki = pid_para_result[1]
        kd = pid_para_result[2]
    else:
        print('find local opt: loss= '+str(last_loss))
    return [kp,ki,kd,now_loss,evaluate_result]


time_start=time.time()
kp=kp_get(exp_val,init_kp_itr,init_speed,init_reachtime_standard)
ki=ki_get(kp,exp_val,init_ki_itr,init_speed,init_ave_standard)
result_of_init=kd_get(kp,ki,exp_val,init_kd_itr,init_speed,init_std_standard)
kd=result_of_init[0]
plt.show()
first_data_result=result_of_init[1]
now_loss=100000
evaluate_result_compara_overshoot=[0]*itr_num
evaluate_result_compara_time = [0] * itr_num
evaluate_result_compara_std = [0] * itr_num
evaluate_result_compara_ave = [0] * itr_num
loss_compara=[0]*itr_num
stop_condition_loss=L1Loss(stop_condition,goal,weight)
print(stop_condition_loss)
for i in range(0,itr_num):
    last_loss=now_loss
    if i==0:
        #zero_one_determiner=get_result_evaluate(first_data_result,stop_condition,exp_val)
        #determin_list=get_determin_list(zero_one_determiner)
        #pid_change_list= turn_change_list(determin_list,delta_kp,delta_ki,delta_kd)
        new_para=try_nearest_parameter(goal,pid_change_list,kp,ki,kd,pid_itr,exp_val,init_speed,window_length,now_loss)
        data_result=new_para[4]
        now_loss = new_para[3]
    elif i>0:
        #zero_one_determiner=get_result_evaluate(data_result,stop_condition,exp_val)
        #determin_list=get_determin_list(zero_one_determiner)
        #pid_change_list= turn_change_list(determin_list,delta_kp,delta_ki,delta_kd)
        new_para = try_nearest_parameter(goal, pid_change_list, kp, ki, kd, pid_itr, exp_val, init_speed, window_length,
                                         now_loss)
        now_loss=new_para[3]
        data_result = new_para[4]
        if now_loss>last_loss:
            print('loacl opt find')
            break
        if now_loss<stop_condition_loss:
            print('stoping condition met')
            break

    if kp==new_para[0] and ki==new_para[1] and kd==new_para[2]:
        print('standard meet')
        break
    kp=new_para[0]
    ki=new_para[1]
    kd=new_para[2]
    loss_compara[i]=new_para[3]
    evaluate_result_compara_overshoot[i]=new_para[4][0]
    evaluate_result_compara_time[i] = new_para[4][1]
    evaluate_result_compara_std[i] = new_para[4][2]
    evaluate_result_compara_ave[i] = new_para[4][3]

time_end = time.time()
time_cost=time_end-time_start
print('time cost')
print(time_cost)
plt.figure()
plt.plot(loss_compara)
plt.title('loss_function')
plt.xlabel('itration')
plt.ylabel('loss value')

plt.figure()
plt.plot(evaluate_result_compara_overshoot)
plt.title('overshoot')
plt.xlabel('itration')
plt.ylabel('overshoot value')

plt.figure()
plt.plot(evaluate_result_compara_time)
plt.title('stable time')
plt.xlabel('itration')
plt.ylabel('time value(s)')

plt.figure()
plt.plot(evaluate_result_compara_std)
plt.title('std after stable point')
plt.xlabel('itration')
plt.ylabel('std value')

plt.figure()
plt.plot(evaluate_result_compara_ave)
plt.title('average after stable point')
plt.xlabel('itration')
plt.ylabel('average value')
plt.plot

plt.show()

