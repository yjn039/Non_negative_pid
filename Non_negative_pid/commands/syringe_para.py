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
    connection = serial.Serial()
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




