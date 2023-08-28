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
from pandas import DataFrame

from commands import connect_flowmeter, init_flowmeter, close_flowmeter,get_measurement
from commands import get_measurement_sf06 ,reset_cable_sf06
from commands.syringe_para import _unit_to_steps,_unit_to_time,_time_to_unit,_time_to_unit,_speed_to_basis, _basis_to_speed,_read_message,sendToArduino,readFromArduino
from commands.syringe_para import getVolume,getSpeed,getAcceleration
from pid_core import pid,pid_stability
# ------------------





def pid_implement(exp_val,pid_itr,pid_para,volume,init_speed):
    global time_per_point,pid_sum_err, pid_now_err, flowm
    pid_sum_err = 0
    pid_now_err = exp_val-init_speed
    sensor_data=[0]*pid_itr
    #sensor_data_num = [0] * pid_itr
    time_start = time.time()
    set_init_speed(init_speed)
    turn_pump_on(volume)
    time.sleep(3)
    now_speed=init_speed
    for i in range(1, pid_itr):
       # sensor_data_num[i] = i
        sensor_data[i] = abs(get_measurement_sf06(flowm, 10))
        loop_time_1=time.time()
        if i==1:
            dt=0.117
        else:
            dt=loop_time_2-loop_time_1
        last_speed = now_speed
        pid_output_result=pid(exp_val,sensor_data[i],pid_para[0],pid_para[1],pid_para[2],pid_now_err,pid_sum_err,dt)

        pid_output_value=pid_output_result[0]
        pid_now_err=pid_output_result[1]
        pid_sum_err = pid_output_result[2]
        now_speed=pid_stability(pid_output_value,last_speed)
        speed_value_forPump = getSpeed(now_speed, pump_unit, syringe_area)
        sendToArduino("<SPEED," + pump_axis + "," + str(speed_value_forPump) + ">")
        loop_time_2=time.time()
        #print(i)
        #print('sensor_value  '+str(sensor_data[i]))
        #print('last speed  '+str(last_speed))
        #print('pid output  '+str(pid_output_result[0]))
        #print('now speed  ' + str(now_speed))
        #print('speed_value'+str(speed_value_forPump))
    time_end = time.time()

    #stop the motor and sensor
    set_init_speed(init_speed)
    sendToArduino("<STOP," + pump_axis + ",1>")
    reset_cable_sf06(flowm)


    #create time line
    time_sum = time_end - time_start
    time_per_point=time_sum/pid_itr

    return sensor_data

def collect_data_point2_excel(kp,ki,kd,pid_itr,exp_val,init_speed):
    global connection,flowm,pump_axis,pump_unit,syringe_area
    ##-\-\-\-\
    ## RUNNING
    ##-/-/-/-/

    # Run the pump for the given volume
    #print('Turn pump on')
    # Wait a bit for the pump to run
    file_path = './raw_data1.xls'
    df = pd.read_excel(file_path)
    sensor_data = pid_implement(exp_val, pid_itr, pid_para, volume, init_speed)
    dataframe = pd.DataFrame(sensor_data)
    dataframe.to_excel(file_path, sheet_name='Sheet1', index=False, header=True)

    ###sensor_data = abs(get_measurement_sf06(flowm, -1))
    ##-\-\-\-\-\-\
    ## TERMINATION
    ##-/-/-/-/-/-/

    # Turn the pump off
    # Stop the connection
    connection.close()
    close_flowmeter(flowm, 'SF06')
    # plt.plot(time_line,sensor_data)
    # plt.show()
    return time_per_point
