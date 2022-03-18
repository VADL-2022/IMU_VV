# -*- coding: utf-8 -*-
"""
Created on Thu Jan 27 17:43:47 2022

@author: kdmen

YOU MUST DOWNLOAD SMSBUS ON THE RPI BEFORE YOU CAN RUN THIS:
    
DATA SHEET FOR THIS IMU
https://www.sparkfun.com/datasheets/Sensors/Accelerometer/LIS331HH.pdf

"""

import smbus
import time
from csv import writer
import time

def append_list_as_row(file_name, list_of_elem):
    # Open file in append mode
    with open(file_name, 'a+', newline='') as write_obj:
        # Create a writer object from csv module
        csv_writer = writer(write_obj)
        # Add contents of list as last row in the csv file
        csv_writer.writerow(list_of_elem)

timestr = time.strftime("%Y%m%d-%H%M%S")
my_log = "LIS_LOG_" + timestr + ".csv"

# Create a new (empty) csv file
with open(my_log, 'w', newline='') as file:
    new_file = writer(file)

# Get I2C bus
bus = smbus.SMBus(1)

#Syntax:
#write_byte_data(self, addr, cmd, val)

# SET THE DATA RATE
###############################################################################
# H3LIS331DL address, 0x18(24)
# Select control register 1, 0x20(32)
#		0x27(39)	Power ON mode, Data output rate = 50 Hz
#					X, Y, Z-Axis enabled
address=0x19
bus.write_byte_data(address, 0x20, 0x27)
# Required Binary: 001 (normal power mode), 00 (50 Hz), 111 (enable XYZ)
# Equivalent Hex: 00100111 -> 0x27
# TOGGLE THE MAXIMUM RANGE
###############################################################################
# H3LIS331DL address, 0x18(24)
# Select control register 4, 0x23(35)
#		0x00(00)	Continuous update,
#Full scale selection = +/-24g: 0x18
#SUBSCALE = +/- 12g: 0x10
#bus.write_byte_data(0x18, 0x23, 0x00)
bus.write_byte_data(address, 0x23, 0x10)

time.sleep(0.5)

avgX=0
avgY=0
avgZ=0
counter=0

offsetX=0
offsetY=0
offsetZ=0

def calibrate(file):
    global offsetX
    global offsetY
    global offsetZ
    with open(file, 'r') as f:
        for line in f:
            pass
        last_line = line
        offsetX,offsetY,offsetZ=list(map(float, last_line.split(",")))

def runOneIter():
    global avgX
    global avgY
    global avgZ
    global counter
    
    # X AXIS
    ###############################################################################
    # H3LIS331DL address, 0x18(24)
    # Read data back from 0x28(40), 2 bytes
    # X-Axis LSB, X-Axis MSB
    data0 = bus.read_byte_data(address, 0x28)
    data1 = bus.read_byte_data(address, 0x29)
    
    # Convert the data
    xAccl = data1 * 256 + data0
    if xAccl > 32767 :
    	xAccl -= 65536
    
    
    # Y AXIS
    ###############################################################################
    # H3LIS331DL address, 0x18(24)
    # Read data back from 0x2A(42), 2 bytes
    # Y-Axis LSB, Y-Axis MSB
    data0 = bus.read_byte_data(address, 0x2A)
    data1 = bus.read_byte_data(address, 0x2B)
    
    # Convert the data
    yAccl = data1 * 256 + data0
    if yAccl > 32767 :
    	yAccl -= 65536
    
    
    # Z AXIS
    ###############################################################################
    # H3LIS331DL address, 0x18(24)
    # Read data back from 0x2C(44), 2 bytes
    # Z-Axis LSB, Z-Axis MSB
    data0 = bus.read_byte_data(address, 0x2C)
    data1 = bus.read_byte_data(address, 0x2D)
    
    # Convert the data
    zAccl = data1 * 256 + data0
    if zAccl > 32767 :
    	zAccl -= 65536
    
    # LOG TO CSV
    ###############################################################################
    # Output data to screen
    #print("Acceleration in X-Axis : %d" %xAccl)
    #print("Acceleration in Y-Axis : %d" %yAccl)
    #print("Acceleration in Z-Axis : %d" %zAccl)

    avgX += xAccl
    avgY += yAccl
    avgZ += zAccl
    counter += 1
    my_accels = [(xAccl-offsetX)/1000.0*9.81, (yAccl-offsetY)/1000.0*9.81, (zAccl-offsetZ)/1000.0*9.81]
    append_list_as_row(my_log, my_accels)

#calibrate("LOG_20220129-182509.csv")
try:
    while True:
        runOneIter()
finally:
    if counter > 0:
        append_list_as_row(my_log, [avgX/counter,avgY/counter,avgZ/counter])
