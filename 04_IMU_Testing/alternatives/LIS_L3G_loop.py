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
my_log = "LIS_L3G_LOG_" + timestr + ".csv"

# Create a new (empty) csv file
with open(my_log, 'w', newline='') as file:
    new_file = writer(file)

# Get I2C bus
LIS_bus = smbus.SMBus(1)
L3G_bus = smbus.SMBus(2)

#Syntax:
#write_byte_data(self, addr, cmd, val)

# LIS
LIS_address=0x19
# Required Binary: 001 (normal power mode), 01 (100 Hz), 111 (enable XYZ) --> Equivalent Hex: 00101111 -> 0x2F
LIS_bus.write_byte_data(LIS_address, 0x10, 0x2F)
#FULL SCALE selection = +/-24g: 0x18
#SUBSCALE = +/- 12g: 0x10
LIS_bus.write_byte_data(LIS_address, 0x23, 0x18)

# L3G
L3G_address=0x6B
# Required Binary: 0001 (Set ODR to 100 Hz), 0111 (Enable everything) --> Equivalent Hex: 00010111 -> 0x17
L3G_bus.write_byte_data(L3G_address, 0x20, 0x17)

time.sleep(0.5)

def runOneIter():
    # LIS331 Accelerometer
    ###############################################################################
    # X AXIS, X-Axis LSB, X-Axis MSB
    data0 = LIS_bus.read_byte_data(LIS_address, 0x28)
    data1 = LIS_bus.read_byte_data(LIS_address, 0x29)
    # Convert the data
    xAccl = data1 * 256 + data0
    if xAccl > 32767:
    	xAccl -= 65536
    # Y AXIS, Y-Axis LSB, Y-Axis MSB
    data0 = LIS_bus.read_byte_data(LIS_address, 0x2A)
    data1 = LIS_bus.read_byte_data(LIS_address, 0x2B)
    # Convert the data
    yAccl = data1 * 256 + data0
    if yAccl > 32767:
    	yAccl -= 65536
    # Z AXIS, Z-Axis LSB, Z-Axis MSB
    data0 = LIS_bus.read_byte_data(LIS_address, 0x2C)
    data1 = LIS_bus.read_byte_data(LIS_address, 0x2D)
    # Convert the data
    zAccl = data1 * 256 + data0
    if zAccl > 32767:
    	zAccl -= 65536
    
    # L3GD20H Gyroscope
    ###############################################################################
    rx = L3G_bus.read_byte_data(L3G_address, 0x29)
    ry = L3G_bus.read_byte_data(L3G_address, 0x2B)
    rz = L3G_bus.read_byte_data(L3G_address, 0x2D)
    
    # LOG TO CSV
    ###############################################################################
    my_vals = [xAccl, yAccl, zAccl, rx, ry, rz]
    append_list_as_row(my_log, my_vals)

try:
    while True:
        runOneIter()
finally:
    pass