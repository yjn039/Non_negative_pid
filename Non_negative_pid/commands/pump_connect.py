import math

import numpy as np
import serial
import matplotlib.pyplot as plt
import serial.tools.list_ports
import time

from commands.__init__ import connect_flowmeter, init_flowmeter, close_flowmeter,get_measurement
from commands.sf06_commands import get_measurement_sf06 ,reset_cable_sf06
from commands.syringe_para import _unit_to_steps,_unit_to_time,_time_to_unit,_time_to_unit,_speed_to_basis, _basis_to_speed,_read_message,sendToArduino,readFromArduino
from commands.syringe_para import getVolume,getSpeed,getAcceleration

def init_pump(pump_unit,syringe_area,pump_axis):
    ports = serial.tools.list_ports.comports()

    # Make the list
    all_ports = []
    for p in ports:
        print(p.device)

    connection = serial.Serial()

    connection.port = 'COM5'  # Set port
    connection.baudrate = 9600  # Set baudrate

    connection.parity = serial.PARITY_NONE  # Enable parity checking
    connection.stopbits = serial.STOPBITS_ONE  # Number of stop bits
    connection.bytesize = serial.EIGHTBITS  # Number of databits
    print(123)
    connection.timeout = 1  # Set a timeout value
    connection.xonxoff = 0  # Disable software control
    connection.rtscts = 0
    print(123)
    connection.open()
    print(123)
    readFromArduino(connection)
    print(123)
    time.sleep(1)
    # Get the ID
    print('Getting pump ID')
    message = sendToArduino("<GETID,0,0.0>")
    print(message)

    # Set the acceleration
    print('Set acceleration')
    acceleration_value = 0  # in uL/min2
    acceleration_value_forPump = getAcceleration(acceleration_value, pump_unit, syringe_area)
    sendToArduino("<ACCEL," + pump_axis + "," + str(acceleration_value_forPump) + ">")
    print(str(acceleration_value_forPump))
    # Set the speed
    print('Set speed')
    speed_value = 0  # in uL/min <--the speed unit in ul/min  u want
    speed_value_forPump = getSpeed(speed_value, pump_unit, syringe_area)
    sendToArduino("<SPEED," + pump_axis + "," + str(speed_value_forPump) + ">")
    print(str(speed_value_forPump))
    ## ================
    ## TO THE FLOWMETER

    # Open the connection

    flowm = connect_flowmeter('COM6')

    # Start the flowmeter
    init_flowmeter(flowm, 'SF06')
    return None

    ##-\-\-\-\
    ## RUNNING
    ##-/-/-/-/

def turn_pump_on(pump_unit,syringe_area,pump_axis):
    # Run the pump for the given volume
    print('Turn pump on')
    volume_value = -100000  # in uL that is the total volume
    volume_value_forPump = getVolume(volume_value, pump_unit, syringe_area)
    sendToArduino("<RUN," + pump_axis + "," + str(volume_value_forPump) + ">")

def turn_pump_off():
    connection = serial.Serial()
    connection.close()
    flowm = connect_flowmeter('COM6')
    close_flowmeter(flowm, 'SF06')
    # plt.plot(time_line,sensor_data)
    # plt.show()
# ------------------
# Convert the volume



