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


def _unit_to_steps(in_value, unit, syringe_area, microsteps=16):

    if unit == 'mm':
        temp_value = in_value

    elif unit == 'mL':
        temp_value = in_value * 1000 / syringe_area

    elif unit == 'µL':
        temp_value = in_value / syringe_area

    # Convert into spins
    out_value = temp_value / 0.8 * 200 * microsteps

    return out_value

# ----------------
# Convert the time
def _unit_to_time(in_value, unit):

    # Initialise the result
    out_value = in_value

    if unit == 's':
        out_value = in_value

    elif unit == 'min':
        out_value = in_value / 60.

    elif unit == 'h':
        out_value = in_value / 60. / 60.

    return out_value

# ----------------
# Convert the time
def _time_to_unit(in_value, unit):

    # Initialise the result
    out_value = in_value

    if unit == 's':
        out_value = in_value

    elif unit == 'min':
        out_value = in_value * 60.

    elif unit == 'h':
        out_value = in_value * 60. * 60.

    return out_value

# -----------------------------
# Convert speed to another unit
def _speed_to_basis(in_value, unit):

    # Split the unit
    volume, time = unit.split('/')

    # Convert the volume
    if volume == 'L':
        temp_value = in_value

    elif volume == 'mL':
        temp_value = in_value / 1000

    elif volume == 'µL':
        temp_value = in_value / 1000 / 1000

    # Convert the time
    out_value = _unit_to_time(temp_value, time)

    return out_value

# -----------------------------
# Convert speed to another unit
def _basis_to_speed(in_value, unit):

    # Split the unit
    volume, time = unit.split('/')

    # Convert the volume
    if volume == 'L':
        temp_value = in_value

    elif volume == 'mL':
        temp_value = in_value * 1000

    elif volume == 'µL':
        temp_value = in_value * 1000 * 1000

    # Convert the time
    out_value = _time_to_unit(temp_value, time)

    return out_value

# ----------------------------------------------
# Read the message coming from the board, if any
def _read_message(board, timeout=1):

    # Define the markers
    startMarker = 60 # <
    endMarker = 62 # >
    midMarker = 44 # ,

    final_string = ""
    x = "z" # Random char to start with

    # Start a timer
    start_time = time.time()
    read_arduino = True

    # Wait for the start character
    while  ord(x) != startMarker and read_arduino:
        x = board.read()

        # Check for empty character
        try:
            test_x = ord(x)
        except:
            x = "z"

        # Check for timeout
        if timeout > 0 and time.time() - start_time >= timeout:
            read_arduino = False

    # Start the processing loop
    while ord(x) != endMarker and read_arduino:

        # Get the next character
        x = board.read()

        # Start the reading process
        if ord(x) != startMarker and ord(x) != endMarker:
            final_string += x.decode()

        # Check for timeout
        if timeout > 0 and time.time() - start_time >= timeout:
            read_arduino = False

    # Return a timeout error if needed
    if not read_arduino:
        final_string = None

    return final_string

# ----------------------------
# Send an input to the Arduino
def sendToArduino(message):

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
def getSpeed(in_speed, unit, syringe_area, microsteps=16):

    # Get the volume to displace
    out_speed = getVolume(in_speed, unit, syringe_area, microsteps=microsteps)

    # Split the unit
    _, time = unit.split('/')

    # Convert the time
    out_speed = _unit_to_time(out_speed, time)

    return out_speed

# ----------------------------------
# Get the value for the acceleration
def getAcceleration(in_accel, unit, syringe_area, microsteps=16):

    # Get the section area of the syringe
    out_accel = getSpeed(in_accel, unit, syringe_area, microsteps=microsteps)

    # Split the unit
    _, time = unit.split('/')

    # Convert the time
    out_accel = _unit_to_time(out_accel, time) # Call it twice to match the time unit

    return out_accel


def pid(exp_val, sensor_data, kp, ki, kd):
    global pid_sum_err
    global pid_last_err
    global pid_now_err
    pid_last_err = pid_now_err

    pid_now_err = exp_val - sensor_data

    pid_sum_err += pid_now_err

    pid_output_value = kp * (pid_now_err) + ki * pid_sum_err + kd* (pid_now_err - pid_last_err)
    #print(pid_output_value)
    pid_output_value=pid_stability(exp_val, pid_output_value)
    return pid_output_value

def pid_stability(exp_val,pid_output_value):
    if pid_output_value>3*exp_val:

        pid_output_value=exp_val

    if pid_output_value<0:

        pid_output_value=1

    return pid_output_value

def pid_parameters_opt_kp(exp_val,error,pid_itr,delay):
    global kp
    global ki
    global kd
    global res_kp
    global pid_sum_err
    global pid_last_err
    global pid_now_err
    break_flag=0
    for i in range(1, 30):
        pid_sum_err = 0
        pid_last_err = 0
        pid_now_err = 0
        sensor_data=[0]*delay*3
        if break_flag==1:
            break
        for i in range(1,delay*3):
           sensor_data[i] = abs(get_measurement_sf06(flowm, -10))
           pid_output_value=pid(exp_val, sensor_data[i], kp, ki, kd)
           accel_rate=abs(pid_output_value-sensor_data[i])*60
           sendToArduino("<SPEED," + pump_axis + "," + str(pid_output_value) + ">")
           sendToArduino("ACCEL," + pump_axis + "," + str(accel_rate) + ">")
           if sensor_data[i] >= exp_val - exp_val * error:
                delay_position=i
                if delay_position<=delay:
                    break_flag=1
                    break
        if np.max(sensor_data)<exp_val - exp_val * error*3:
           kp+=res_kp*3
        else:
           kp+=res_kp

    kp = kp * 0.9
    return kp


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
exp_val=200 #ul/min
kp=0
ki=0
kd=0
res_kp=0.1
res_ki=0.01
res_kd=0.1
error=0.05
delay=15
pid_sum_err=0
pid_last_err=0
pid_now_err=0
pid_itr_para_opt=150
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
volume_value = -100000 # in uL that is the total volume
volume_value_forPump = getVolume(volume_value, pump_unit, syringe_area)
sendToArduino("<RUN," + pump_axis + "," + str(volume_value_forPump) + ">")
print(str(volume_value_forPump))
# Wait a bit for the pump to run
kp=pid_parameters_opt_kp(exp_val,error,pid_itr,delay)
print(kp)
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