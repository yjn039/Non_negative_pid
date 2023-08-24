import math

import numpy as np
import serial
import matplotlib.pyplot as plt
import serial.tools.list_ports
import time
import scipy.signal as signal
import xlwt
import pandas as pd
import statsmodels.tsa.api as smt

def np_move_avg(a, n, mode="same"):  # moving average filter
    return (np.convolve(a, np.ones((n,)) / n, mode=mode))




def find_stable(sensor_data_moveave,sensor_data, time_point, exp_val):
    after_overshoot = 0
    osc_period_list = osc_period_anaylsis(exp_val, sensor_data_moveave)

    overshoot = find_overshoot(sensor_data_moveave, exp_val)[0]  # find overshoot function will return[overshoot_val,overshoot_position]
    if overshoot != 0 and len(osc_period_list)>1:
        after_overshoot = osc_period_list[1]
    #print('after_overshoot_position')
    #print(after_overshoot)

    # noise amp
    moveave_after_overshoot = sensor_data_moveave[after_overshoot:]
    signal_after_overshoot = sensor_data[after_overshoot:len(sensor_data_moveave)]

    noise = moveave_after_overshoot - signal_after_overshoot
    noise_std = np.std(noise)
    noise_rms = np.sqrt(np.mean(noise ** 2)) * np.sqrt(2) / exp_val

    if np.std(sensor_data_moveave-exp_val)>noise_std:
#find average period via extreme value to determin the window length
      osc_period=round((osc_period_list[len(osc_period_list)-1]-osc_period_list[0])/len(osc_period_list))
      #print('sssss'+osc_period)
      osc_period_ave=np.average(osc_period)
      #print(osc_period_ave)
      window_length=int(round(osc_period_ave*2))
      #print('m-std_window_length')
      #print(window_length)


#m-std
      stab_position=-1
      f = pd.Series(sensor_data_moveave)
      f = f[f.index>=after_overshoot]
      m_std=f.rolling(window=window_length, center=False).std()
      #print('length m std')
      #print(len(m_std))
      #plt.figure()
     # plt.plot(m_std)



      std_smaller_than_noise=[0]*len(m_std)
      for i in range(after_overshoot,len(m_std)):
         if m_std[i]<noise_rms*exp_val:
             std_smaller_than_noise[i]=1
     # plt.figure()
     # plt.plot(std_smaller_than_noise)

      for i in range(0,len(std_smaller_than_noise)):
        if std_smaller_than_noise[i]==1 and i in osc_period_list :
          after_stab_point=std_smaller_than_noise[i:]
          after_stab_point_determin=[k for k in after_stab_point if k == 0]
          if len(after_stab_point_determin)==0:
                  stab_position=i
                  #print(stab_position)
                  break

    else:
        stab_position=osc_period_list[1]
    return stab_position


def find_noise_after_stab(sensor_data, stab_position):
    std_value = np.std(sensor_data[stab_position:])
    if stab_position==-1:
        std_value=-1

    return std_value


def find_time_to_stab(stab_position, time_per_point):
    time_stab = time_per_point * stab_position
    if stab_position==-1:
        time_stab=-1

    return time_stab


def find_ave_after_stab(sensor_data, stab_position):
    average = np.mean(sensor_data[stab_position:])
    if stab_position==-1:
        average=-1

    return average

def osc_period_anaylsis(exp_val,sensor_data_moveave):
     shifted_data=sensor_data_moveave-exp_val
     signed_data=shifted_data[:-1]*shifted_data[1:]
     oscillation_times_list=np.where(signed_data<0)[0]

     return oscillation_times_list



def find_overshoot(sensor_data, exp_val):
    position=-1
    overshoot_value=-1

    osc_period_list = osc_period_anaylsis(exp_val,sensor_data)
    #print(osc_period_list)
    if len(osc_period_list)>1:
        overshoot = np.max(sensor_data[:osc_period_list[1]])
    else:
        overshoot=np.max(sensor_data)
    if overshoot>=exp_val+2*np.std(sensor_data[round(1/2*len(sensor_data)):]):
        if len(osc_period_list) > 1:
            position=np.argmax(np.array(sensor_data[:osc_period_list[1]]))
        else:
            position = np.argmax(np.array(sensor_data))
        overshoot_value=overshoot
    return (overshoot_value, position)


def find_overshoot_sensor_data(sensor_data,sensor_data_moveave,window_length,stab_point,exp_val):
    filted_overshoot = find_overshoot(sensor_data_moveave, exp_val)
    #print(filted_overshoot)
    if stab_point==-1:
        if filted_overshoot[1]!=-1:
           overshoot_value = np.max(sensor_data[:filted_overshoot[1]+window_length])
           overshoot_position = np.argmax(np.array(sensor_data[:filted_overshoot[1]+window_length]))
        else:
           overshoot_value=-1
           overshoot_position=0
    else:
        overshoot_value=np.max(sensor_data[:stab_point])
        overshoot_position=np.argmax(np.array(sensor_data[:stab_point]))

    return [overshoot_value,overshoot_position]



def evaluate(sensor_data,window_length, exp_val,time_per_point,graph,kp,ki,kd):
    data_length=len(sensor_data)
    sensor_data_filted = np_move_avg(sensor_data, window_length, mode='valid')
    time_line = [0] * len(sensor_data)
    for i in range(1, data_length):
        time_line[i] = i * time_per_point
    sensor_data=sensor_data[0:len(sensor_data_filted)]
    stab = find_stable(sensor_data_filted,sensor_data,time_per_point,exp_val)
    overshoot=find_overshoot(sensor_data_filted, exp_val)
    #overshoot = find_overshoot_sensor_data(sensor_data,sensor_data_filted,window_length,stab,exp_val)
    stab_time = find_time_to_stab(stab, time_per_point)
    stab_var = find_noise_after_stab(sensor_data, stab)
    stab_ave = find_ave_after_stab(sensor_data, stab)
    data_result = [overshoot[0], stab_time, stab_var, stab_ave]
    print('data_result')
    print(data_result)

    if graph==1:
        print('stab')
        print(stab)
        print('data_result')
        print(data_result)
        font = {'family': 'serif',
                'color': 'darkred',
                'weight': 'normal',
                'size': 10}
        if stab == -1:
            result_str = 'unstable'
        else:
            result_str = 'stable'
        text_position_x = round(0.2 * max(time_line))
        plt.figure()
        plt.xlabel('time(s)')
        plt.ylabel('Flow value(ul/min)')
        #plt.title('Raw data and valid data')
        plt.axhline(exp_val, color='r', label='expected value')
        plt.plot(time_line[0:len(sensor_data)], sensor_data, label='raw data')
        plt.plot(
            time_line[round(1 / 2 * window_length):len(sensor_data_filted) + round(1 / 2 * window_length)],
            sensor_data_filted[0:len(sensor_data_filted)], label='True value')
        plt.plot(time_line[stab + round(1 / 2 * window_length)], sensor_data_filted[stab], '*',
                 label='stable point')
        plt.axvline(time_line[stab + round(1 / 2 * window_length)], color='g', label='stable position', dashes=[4, 2])
        plt.plot(time_line[overshoot[1]+round(1/2*window_length)], sensor_data_filted[overshoot[1]], '*', label='overshoot')
        plt.axvline(time_line[overshoot[1]+round(1/2*window_length)], color='r', label='overshoot position', dashes=[4, 2])
        plt.text(text_position_x, 0, 'Overshoot value: ' + str(overshoot[0]) + 'ul/min', fontdict=font)
        plt.text(text_position_x, 25, 'Stable point: ' + str(stab_time) + ' s', fontdict=font)
        plt.text(text_position_x, 50, 'Average value after stable: ' + str(stab_ave) + ' ul/min',
                 fontdict=font)
        plt.text(text_position_x, 75, 'STD after stable: ' + str(stab_var), fontdict=font)
        plt.title('Data after evaluate' + '  # Result: '+' # ' + (result_str)+'   pid_para'+str(kp)+','+str(ki)+','+str(kd))
        plt.legend()


    return data_result

