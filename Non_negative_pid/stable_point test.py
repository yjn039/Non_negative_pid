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

from commands import connect_flowmeter, init_flowmeter, close_flowmeter,get_measurement
from commands import get_measurement_sf06 ,reset_cable_sf06
from commands.syringe_para import _unit_to_steps,_unit_to_time,_time_to_unit,_time_to_unit,_speed_to_basis, _basis_to_speed,_read_message,sendToArduino,readFromArduino
from commands.syringe_para import getVolume,getSpeed,getAcceleration
from evaluate import np_move_avg,evaluate,osc_data_anaylsis,osc_period_anaylsis,find_overshoot
from pid_core import pid,pid_stability

file_path='./raw_data.xls'
config_number=1
dataset_number=4
sheet_name_list=['200group','300group','400group','500group']
dataset_list=['data1','data2','data3','data4','data5','data6']
df= pd.read_excel(file_path,sheet_name=sheet_name_list[config_number-1])
sensor_data=df[dataset_list[dataset_number-1]]



pid_itr=len(sensor_data)
exp_val_list=[200,300,400,500]
exp_val=exp_val_list[config_number-1]
moveave_filter_length=20
time_point=0.1117


print('moveave_filter_length')
print(moveave_filter_length)
print('data period')
print(time_point)

time_line = [0] * pid_itr
for i in range(1, pid_itr):
    time_line[i] = i * time_point
#use moving average filer
sensor_data_moveave=np_move_avg(sensor_data, moveave_filter_length, mode='valid')
print(len(sensor_data_moveave))


    #find average noise Amp after overshoot
    #cut overshoot data
after_overshoot=0
osc_period_list=osc_period_anaylsis(exp_val,sensor_data_moveave)

overshoot=find_overshoot(sensor_data_moveave,exp_val)[0]#find overshoot function will return[overshoot_val,overshoot_position]
if overshoot!=0:
       after_overshoot=osc_period_list[1]

print('after_overshoot_position')
print(after_overshoot)

#noise amp
moveave_after_overshoot=sensor_data_moveave[ after_overshoot:]
signal_after_overshoot=sensor_data[ after_overshoot:len(sensor_data_moveave)]


noise=moveave_after_overshoot-signal_after_overshoot
noise_std=np.std(noise)
noise_rms=np.sqrt(np.mean(noise**2))*np.sqrt(2)/exp_val
print('noise_rms')
print(noise_rms)

plt.figure()
plt.plot(time_line[after_overshoot:len(noise) + after_overshoot], noise)
plt.xlabel('time(s)')
plt.ylabel('noise')
plt.title('Noise after overshoot(delete overshoot part)')



print(np.std(sensor_data_moveave[after_overshoot:]-exp_val))
print(noise_std)
if np.std(sensor_data_moveave[after_overshoot:]-exp_val)>noise_std:
#find average period via extreme value to determin the window length
    osc_period=round((osc_period_list[len(osc_period_list)-1]-osc_period_list[0])/len(osc_period_list))
    osc_period_ave=np.average(osc_period)
    window_length=int(round(osc_period_ave*2))
    print('m-std_window_length')
    print(window_length)


#m-std
    stab_position=0
    f = pd.Series(sensor_data_moveave)
    f = f[f.index>=after_overshoot]
    m_std=f.rolling(window=window_length, center=False).std()
    print('length m std')
    print(len(m_std))



    plt.figure()
    plt.plot(time_line[0:len(m_std)],m_std)
    plt.title('m-std')
    plt.ylabel('m-std value')
    plt.xlabel('time (s)')
    std_smaller_than_noise=[0]*len(m_std)
    for i in range(after_overshoot,len(m_std)):
       if m_std[i]<noise_rms*exp_val:
           std_smaller_than_noise[i]=1
    plt.figure()
    plt.plot(time_line[0:len(std_smaller_than_noise)],std_smaller_than_noise)
    plt.xlabel('time(s)')
    plt.ylabel('yes=1,no=0')
    plt.title('m-std value equal or greater than （average noise Amp)*exp_val')
    for i in range(0,len(std_smaller_than_noise)):
      if std_smaller_than_noise[i]==1 and i in osc_period_list :
        after_stab_point=std_smaller_than_noise[i:]
        after_stab_point_determin=[k for k in after_stab_point if k == 0]
        if len(after_stab_point_determin)==0:
                stab_position=i
                print(stab_position)
                break

    if noise_rms>0.5:
       stab_position=0
    print('std_position')
    print(stab_position)
    print(f[i])



 #acf
    stdstd = np.std(f[stab_position:len(f)])
    if stdstd>20:
        m_std_position=0
    print('stdstd')
    print(stdstd)
    position=0
    if noise_rms>0.5 and stab_position==0:
      print('apply acf')
      max_acf = [0] * (len(sensor_data_moveave))
      for i in range(0, len(sensor_data_moveave)):
        acf = smt.stattools.acf(sensor_data_moveave[i:len(sensor_data_moveave)],
                                nlags=round(1 / 2 * len(sensor_data_moveave[i:len(sensor_data_moveave)])))
        for j in range(0, len(acf)):
            if acf[j] < 0:
                break
        acf_m = acf[j:len(acf)]
        acf_ex = acf_m[signal.argrelextrema(acf_m, np.greater)]
        acf_ex_position = signal.argrelextrema(acf_m, np.greater)[0]
        acf_min = acf_m[signal.argrelextrema(acf_m, np.less)]
        acf_min_position = signal.argrelextrema(acf_m, np.less)[0]
        minus_n = False
        minus_p = False
        if len(acf_ex) == len(acf_min) or len(acf_ex) + 1 == len(acf_min):
            if len(acf_ex)>1:
                minus_p = all(acf_ex[m] > acf_ex[m + 1] for m in range(0, len(acf_ex) - 1))
            elif len(acf_min) > 1:
                minus_n = all(acf_min[m] < acf_min[m + 1] for m in range(0, len(acf_min) - 1))
            elif len(acf_ex)==1:
                    minus_p=True
            elif len(acf_min)==1:
                    minus_n=True
            if minus_p == True and minus_n == True :
                for j in range(0, len(acf_ex)):
                    if acf_min_position[j] < acf_ex_position[j]:
                        if acf_ex[j] > 0 and acf_min[j] < 0:
                            #sum_acf[i] = sum_acf[i] + acf_ex[j]
                            max_acf[i]=np.max(acf_ex)
                        else:
                            break
                    elif j == len(acf_min_position):
                        break
                    else:
                        break

      stab_position = np.argmax(max_acf)

      plt.figure()
      plt.title('autocorrelation from stable point 2 end')
      plt.xlabel('time delay')
      plt.ylabel('autocorrelation coefficient')
      plt.axhline(0, color='r')
      acf_position = smt.stattools.acf(sensor_data_moveave[stab_position:len(sensor_data_moveave)],
                                       nlags=round(1 / 2 * len(sensor_data_moveave[stab_position:len(sensor_data_moveave)])))
      plt.plot(time_line[0:len(acf_position)], acf_position)

      plt.figure()
      plt.title('max of the extreme point for all autocorr function')
      plt.xlabel('time(s)')
      plt.ylabel('max of extreme point value')
      plt.plot(time_line[0:len(max_acf)], max_acf)
    else:
        max_acf = [0] * (len(sensor_data_moveave))


else:
    stab_position=osc_period_list[2]




print('acf postion')
print(stab_position)





plt.figure()
plt.axhline(exp_val, color='r')
lenth_sensor_data=range(0,len(sensor_data),1)
plt.plot(time_line[0:len(sensor_data)],sensor_data,label='raw data')
plt.plot(time_line[round(1/2*moveave_filter_length):len(sensor_data_moveave)+round(1/2*moveave_filter_length)], sensor_data_moveave[0:len(sensor_data_moveave)],label='moving average')
plt.plot(time_line[stab_position+round(1/2*moveave_filter_length)],sensor_data_moveave[stab_position],'*',label='stable point')

plt.legend()
plt.xlabel('time(s)')
plt.ylabel('output value')
plt.title('signal')





plt.show()
