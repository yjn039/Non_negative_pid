import math

import numpy as np
import serial
import matplotlib.pyplot as plt
import serial.tools.list_ports
import time

from commands import connect_flowmeter, init_flowmeter, close_flowmeter,get_measurement
from commands import get_measurement_sf06 ,reset_cable_sf06
# ------------------
# Convert the volume


def pid_parameters_opt_kp(exp_val,error,pid_itr,delay):
    global kp
    global ki
    global kd
    global res_kp
    global pid_sum_err
    global pid_last_err
    global pid_now_err
    break_flag=0
    for j in range(1, 50):
        pid_sum_err = 0
        pid_last_err = 0
        pid_now_err = 0
        sensor_data=[0]*delay*3
        if break_flag==1:
            break
            sendToArduino("<STOP," + pump_axis + "," + str(1) + ">")
            time.sleep(1)
            volume_value = -100000  # in uL that is the total volume
            volume_value_forPump = getVolume(volume_value, pump_unit, syringe_area)
            sendToArduino("<RUN," + pump_axis + "," + str(volume_value_forPump) + ">")
        for i in range(1,delay*3):
           sensor_data[i] = abs(get_measurement_sf06(flowm, -10))
           pid_output_value=pid(exp_val, sensor_data[i], kp, ki, kd)
           accel_rate=abs(pid_output_value-sensor_data[i])*60
           sendToArduino("<SPEED," + pump_axis + "," + str(pid_output_value) + ">")
           sendToArduino("ACCEL," + pump_axis + "," + str(accel_rate) + ">")
           if sensor_data[i] >= 1.2*exp_val:
                    break_flag=1
                    break
        if np.max(sensor_data)<exp_val:
           kp+=res_kp*5
           print(np.max(sensor_data))
        if np.max(sensor_data)<exp_val*1.1:
           kp+=res_kp*3
           print(np.max(sensor_data))
        else:
           kp+=res_kp
           print(np.max(sensor_data))
    kp = kp
    return kp

def pid_parameters_opt_ki(exp_val,error,pid_itr,delay):
    global kp
    global ki
    global kd
    global res_ki
    global pid_sum_err
    global pid_last_err
    global pid_now_err
    for j in range(1,50):
        ki += res_ki
        pid_sum_err = 0
        pid_last_err = 0
        pid_now_err = 0
        sensor_data_stat = [0] * (pid_itr_para_opt - delay)
        sendToArduino("<STOP," + pump_axis + "," + str(1) + ">")
        time.sleep(1)
        volume_value = -100000  # in uL that is the total volume
        volume_value_forPump = getVolume(volume_value, pump_unit, syringe_area)
        sendToArduino("<RUN," + pump_axis + "," + str(volume_value_forPump) + ">")
        for i in range(1,pid_itr_para_opt):
           sensor_data = abs(get_measurement_sf06(flowm, -10))
           pid_output_value=pid(exp_val, sensor_data, kp, ki, kd)
           accel_rate=abs(pid_output_value-sensor_data)*60
           sendToArduino("<SPEED," + pump_axis + "," + str(pid_output_value) + ">")
           sendToArduino("ACCEL," + pump_axis + "," + str(accel_rate) + ">")
           if i>delay:
               sensor_data_stat[i-delay] = sensor_data
        var_sum=0
        for k in range (1,len(sensor_data_stat)):
            var_sum+=pow((sensor_data_stat[k]-exp_val),2)
            variance=var_sum/len(sensor_data_stat)
        print(variance)
        if variance<pow((exp_val*error*3),2):
            print(variance)
            break
    return ki

def pid_parameters_opt_kd(exp_val,error,pid_itr,delay):
    global kp
    global ki
    global kd
    global res_kd
    global pid_sum_err
    global pid_last_err
    global pid_now_err
    break_flag = 0
    pid_cal_delay_determin = 0
    pid_cal_delay=0
    for j in range(1,delay*3):
        pid_sum_err = 0
        pid_last_err = 0
        pid_now_err = 0
        if break_flag==1:
            break
        kd += res_kd
        sendToArduino("<STOP," + pump_axis + "," + str(1) + ">")
        time.sleep(1)
        volume_value = -100000  # in uL that is the total volume
        volume_value_forPump = getVolume(volume_value, pump_unit, syringe_area)
        sendToArduino("<RUN," + pump_axis + "," + str(volume_value_forPump) + ">")
        for i in range(1,pid_itr_para_opt):
           sensor_data = abs(get_measurement_sf06(flowm, -10))
           pid_output_value=pid(exp_val, sensor_data, kp, ki, kd)
           accel_rate=abs(pid_output_value-sensor_data)*60
           sendToArduino("<SPEED," + pump_axis + "," + str(pid_output_value) + ">")
           sendToArduino("ACCEL," + pump_axis + "," + str(accel_rate) + ">")
           if sensor_data>= exp_val - exp_val * error:
               delay_position = i
               if delay_position <= delay:
                   break_flag = 1
                   break
    return kd
##-\-\-\-\-\-\-\
## INITIALISATION
##-/-/-/-/-/-/-/

# Get the list of ports
ports = serial.tools.list_ports.comports()

# Make the list
all_ports = []
for p in ports:
    print(p.device)

## ===========
## TO THE PUMP

# Set the properties of the pump
pump_axis = '1' # Use string, not integer
pump_unit = 'µL/min'
syringe_area = 100*math.pi # in mm2  syringe(r)^2*pi
exp_val=100 #ul/min
kp=3
ki=0.0
kd=0
res_kp=0.1
res_ki=0.01
res_kd=0.1
error=0.01
delay=30
pid_sum_err=0
pid_last_err=0
pid_now_err=0
pid_itr_para_opt=100
pid_itr=200
pid_cal_delay=20
sensor_data_sum=0
sensor_data=[0]*pid_itr
sensor_data_num=[0]*pid_itr
# Start the connection
connection = serial.Serial()

connection.port = 'COM5' # Set port
connection.baudrate = 9600 # Set baudrate

connection.parity = serial.PARITY_NONE # Enable parity checking
connection.stopbits = serial.STOPBITS_ONE # Number of stop bits
connection.bytesize = serial.EIGHTBITS # Number of databits

connection.timeout = 1 # Set a timeout value
connection.xonxoff = 0 # Disable software control
connection.rtscts = 0

connection.open()

readFromArduino(connection)

time.sleep(1)

# Get the ID
print('Getting pump ID')
message = sendToArduino("<GETID,0,0.0>")
print(message)

# Set the acceleration
print('Set acceleration')
acceleration_value = 0 # in uL/min2
acceleration_value_forPump = getAcceleration(acceleration_value, pump_unit, syringe_area)
sendToArduino("<ACCEL," + pump_axis + "," + str(acceleration_value_forPump) + ">")
print(str(acceleration_value_forPump))
# Set the speed
print('Set speed')
speed_value = exp_val # in uL/min <--the speed unit in ul/min  u want
speed_value_forPump = getSpeed(speed_value, pump_unit, syringe_area)
sendToArduino("<SPEED," + pump_axis + "," + str(speed_value_forPump) + ">")
print(str(speed_value_forPump))
## ================
## TO THE FLOWMETER

# Open the connection


flowm = connect_flowmeter('COM4')

# Start the flowmeter
init_flowmeter(flowm, 'SF06')

##-\-\-\-\
## RUNNING
##-/-/-/-/

# Run the pump for the given volume
print('Turn pump on')
volume_value = -1000000 # in uL that is the total volume
volume_value_forPump = getVolume(volume_value, pump_unit, syringe_area)
sendToArduino("<RUN," + pump_axis + "," + str(volume_value_forPump) + ">")
print(str(volume_value_forPump))
# Wait a bit for the pump to run
kp=pid_parameters_opt_kp(exp_val,error,pid_itr,delay)
print(kp)
pid_sum_err = 0
pid_last_err = 0
pid_now_err = 0
ki=pid_parameters_opt_ki(exp_val,error,pid_itr,delay)
print(ki)
pid_sum_err = 0
pid_last_err = 0
pid_now_err = 0
#kd=pid_parameters_opt_kd(exp_val,error,pid_itr,delay)
kd=0.5*ki
kp=1.2*kp
print(kd)
pid_sum_err = 0
pid_last_err = 0
pid_now_err = 0
sendToArduino("<STOP," + pump_axis + "," + str(1) + ">")
time.sleep(10)
volume_value = -100000 # in uL that is the total volume
volume_value_forPump = getVolume(volume_value, pump_unit, syringe_area)
sendToArduino("<RUN," + pump_axis + "," + str(volume_value_forPump) + ">")
for i in range(1,pid_itr):
    sensor_data[i] = abs(get_measurement_sf06(flowm, -10))
    sensor_data_num[i]=i
    if i>pid_cal_delay:
        sensor_data_sum+=sensor_data[i]

    pid_output_value=pid(exp_val,sensor_data[i],kp,ki,kd)
    accel_rate=abs(pid_output_value-sensor_data[i])*60
    sendToArduino("<SPEED," + pump_axis + "," + str(pid_output_value) + ">")
    sendToArduino("ACCEL," + pump_axis + "," + str(accel_rate) + ">")


sensor_data_ave=sensor_data_sum/(pid_itr-pid_cal_delay)
print('average')
print(sensor_data_ave)

###sensor_data = abs(get_measurement_sf06(flowm, -1))
##-\-\-\-\-\-\
## TERMINATION
##-/-/-/-/-/-/

# Turn the pump off
print('Turning pump off')
sendToArduino("<STOP," + pump_axis + ",1>")
reset_cable_sf06(flowm)
# Stop the connection
connection.close()
close_flowmeter(flowm, 'SF06')
plt.plot(sensor_data_num,sensor_data)
plt.show()
print(kp,ki,kd)