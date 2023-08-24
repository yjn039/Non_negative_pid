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


def sendToArduino(message):
    global connection
    connection.write(bytes(message, 'utf-8'))
    connection.flushInput()
    newMessage = readFromArduino(connection)

    return newMessage

# --------------------------------
# Read any output from the Arduino
def readFromArduino(board, timeout=-1):

    # Start a timer
    start_time = time.time()
    read_arduino = True

    # Start the waiting loop
    while board.inWaiting() == 0 and read_arduino:

        # Check for timeout
        if timeout > 0 and time.time() - start_time >= timeout:
            read_arduino = False

    # Read the incoming message
    if read_arduino:
        return _read_message(board)

    else:
        return None

# ------------------------
# Get the volume to inject
def getVolume(in_volume, unit, syringe_area, microsteps=16):

    # Split the unit
    volume, _ = unit.split('/')

    # Convert the volume into steps
    out_volume = _unit_to_steps(in_volume, volume, syringe_area, microsteps=microsteps)

    return out_volume

# ---------------------------
# Get the value for the speed

##-\-\-\-\-\-\-\
## INITIALISATION
##-/-/-/-/-/-/-/

# Get the list of ports
ports = serial.tools.list_ports.comports()

# Make the list
all_ports = []
for p in ports:
    print(p.device)

def turn_pump_on(volume):
    volume_value = volume # in uL that is the total volume
    volume_value_forPump = getVolume(volume_value, 'µL/min', 100 * math.pi)
    sendToArduino("<RUN," + '1' + "," + str(volume_value_forPump) + ">")
    return None


def set_init_speed(init_speed):
    speed_value_forPump = getSpeed(init_speed, 'µL/min', 100 * math.pi)
    sendToArduino("<SPEED," + '1' + "," + str(init_speed) + ">")
    return None



def pid_implement(exp_val,pid_itr,pid_para,volume,init_speed):
    global time_per_point,pid_sum_err, pid_now_err, flowm
    pid_sum_err = 0
    pid_now_err = 0
    time_sum=0
    sensor_data=[0]*pid_itr
    #sensor_data_num = [0] * pid_itr
    time_start = time.time()
    set_init_speed(init_speed)
    turn_pump_on(volume)
    time.sleep(3)
    now_speed=0
    for i in range(1, pid_itr):
       # sensor_data_num[i] = i

        sensor_data[i] = abs(get_measurement_sf06(flowm, -10))
        if i>1:
           last_speed = now_speed
           pid_output_result=pid(exp_val,sensor_data[i],pid_para[0],pid_para[1],pid_para[2],pid_now_err,pid_sum_err)
           pid_output_value=pid_output_result[0]
           pid_now_err=pid_output_result[1]
           pid_sum_err = pid_output_result[2]
           #now_speed=pid_output_value+last_speed
           now_speed=pid_stability(pid_output_value+last_speed)
           speed_value_forPump = getSpeed(now_speed, pump_unit, syringe_area)
           sendToArduino("<SPEED," + pump_axis + "," + str(speed_value_forPump) + ">")
    time_end = time.time()

    #stop the motor and sensor
    sendToArduino("<STOP," + pump_axis + ",1>")
    reset_cable_sf06(flowm)


    #create time line
    time_sum = time_end - time_start
    time_per_point=time_sum/pid_itr
    return sensor_data

def collect_data_point2_excel(kp,ki,kd,pid_itr,exp_val,init_speed):
    global connection,flowm,pump_axis,pump_unit,syringe_area
    pump_axis = '1'  # Use string, not integer
    pump_unit = 'µL/min'
    syringe_area = 100 * math.pi  # in mm2  syringe(r)^2*pi
    pid_para = [kp, ki, kd]
    # pid execution times in loop
    volume = -300000  # total volume
    speed_value = init_speed  # init speed

    modes = ['full', 'same', 'valid']
    # Start the connection
    connection = serial.Serial()

    connection.port = 'COM5'  # Set port
    connection.baudrate = 9600  # Set baudrate

    connection.parity = serial.PARITY_NONE  # Enable parity checking
    connection.stopbits = serial.STOPBITS_ONE  # Number of stop bits
    connection.bytesize = serial.EIGHTBITS  # Number of databits

    connection.timeout = 1  # Set a timeout value
    connection.xonxoff = 0  # Disable software control
    connection.rtscts = 0

    connection.open()

    readFromArduino(connection)

    time.sleep(1)

    # Get the ID
    #print('Getting pump ID')
    message = sendToArduino("<GETID,0,0.0>")
    #print(message)

    # Set the acceleration
    #print('Set acceleration')
    acceleration_value = 1000000  # in uL/min2
    acceleration_value_forPump = getAcceleration(acceleration_value, pump_unit, syringe_area)
    sendToArduino("<ACCEL," + pump_axis + "," + str(acceleration_value_forPump) + ">")
    #print(str(acceleration_value_forPump))
    # Set the speed
    #print('Set speed')
    # in uL/min <--the speed unit in ul/min  u want
    speed_value_forPump = getSpeed(speed_value, pump_unit, syringe_area)
    sendToArduino("<SPEED," + pump_axis + "," + str(speed_value_forPump) + ">")
    #print(str(speed_value_forPump))
    ## ================
    ## TO THE FLOWMETER

    # Open the connection

    flowm = connect_flowmeter('COM6')

    # Start the flowmeter
    init_flowmeter(flowm, 'SF06')

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