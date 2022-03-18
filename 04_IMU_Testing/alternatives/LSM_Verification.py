# -*- coding: utf-8 -*-
"""
Created on Thu Feb 17:43:47 2022

@author: kdmen

"""

import smbus
from time import time, strftime
from csv import writer
from numpy import array

def append_list_as_row(file_name, list_of_elem):
    # Open file in append mode
    with open(file_name, 'a+', newline='') as write_obj:
        # Create a writer object from csv module
        csv_writer = writer(write_obj)
        # Add contents of list as last row in the csv file
        csv_writer.writerow(list_of_elem)

#timestr = strftime("%Y%m%d-%H%M%S")
my_log = "LSM_drop_calibrated.csv"  #03_LSM_TESTING_LOG

# Create a new (empty) csv file
with open(my_log, 'w', newline='') as file:
    new_file = writer(file)

# Get I2C bus
bus = smbus.SMBus(1)
address = 0x6B

#Syntax:
#write_byte_data(self, addr, cmd, val)

# HIGH PERFORMANCE MODE IS ON BY DEFAULT
# SET THE DATA RATE
###############################################################################
# Required Binary: 0101 (Set ODR to 208 Hz), 01 (+/- 16g), 11 (50 Hz Anti Aliasing) --> Equivalent Hex: 01010111 -> 0x57
# @Default anti-alieasing of 400 Hz: 01010100 --> 0x54
bus.write_byte_data(address, 0x10, 0x54)
# Required Binary: 0101 (Set ODR to 208 Hz), 00 (245 dps), 10 (Set SA0 high) --> Equivalent Hex: 01010010 -> 0x52
bus.write_byte_data(address, 0x11, 0x52)

#Noise is in units of micro-g * sqrt(Hz)

def get_accels():
    address = 0x6B
    
    axL = bus.read_byte_data(address, 0x28)
    axH = bus.read_byte_data(address, 0x29)
    ax = axH * 256 + axL
    if ax > 32767 :
        ax -= 65536
        
    ayL = bus.read_byte_data(address, 0x2A)
    ayH = bus.read_byte_data(address, 0x2B)
    ay = ayH * 256 + ayL
    if ay > 32767 :
        ay -= 65536
        
    azL = bus.read_byte_data(address, 0x2C)
    azH = bus.read_byte_data(address, 0x2D)
    az = azH * 256 + azL
    if az > 32767 :
        az -= 65536
        
    return [ax, ay, az]


def get_accel_bytes():
    address = 0x6B
    
    axL = bus.read_byte_data(address, 0x28)
    axH = bus.read_byte_data(address, 0x29)
        
    ayL = bus.read_byte_data(address, 0x2A)
    ayH = bus.read_byte_data(address, 0x2B)
        
    azL = bus.read_byte_data(address, 0x2C)
    azH = bus.read_byte_data(address, 0x2D)
        
    return [axL, axH, ayL, ayH, azL, azH]


def get_gyro():
    address = 0x6B
    
    wxL = bus.read_byte_data(address, 0x22)
    wxH = bus.read_byte_data(address, 0x23)
    wx = wxH * 256 + wxL
    if wx > 32767 :
        wx -= 65536

    wyL = bus.read_byte_data(address, 0x24)
    wyH = bus.read_byte_data(address, 0x25)
    wy = wyH * 256 + wyL
    if wy > 32767 :
        wy -= 65536
        
    wzL = bus.read_byte_data(address, 0x26)
    wzH = bus.read_byte_data(address, 0x27)
    wz = wzH * 256 + wzL
    if wz > 32767 :
        wz -= 65536
        
    return [wx, wy, wz]


def get_z_accel():
    address = 0x6B
    azL = bus.read_byte_data(address, 0x2C)
    azH = bus.read_byte_data(address, 0x2D)
    az = azH * 256 + azL
    if az > 32767 :
        az -= 65536
    return az


def log_z_accel(time_of_startup):
    azL = bus.read_byte_data(address, 0x2C)
    azH = bus.read_byte_data(address, 0x2D)
    az = azH * 256 + azL
    if az > 32767 :
        az -= 65536

    timestamp = time() - time_of_startup

    my_accels = array(az) #* 4.0 / 32768.0
    my_vals = list(my_accels)
    my_vals.extend([timestamp])

    append_list_as_row(my_log, my_vals)


def runOneIter(time_of_startup):
    # CONVERSION FACTOR TO M/S^2:
    ac_conv_factor = 0.00482283
    # CONVERSION FACTOR TO DEG:
    g_conv_factor = 0.00072
    # ^^Both were empirically determined
    
    wxL = bus.read_byte_data(address, 0x22)
    wxH = bus.read_byte_data(address, 0x23)
    wx = wxH * 256 + wxL
    if wx > 32767 :
        wx -= 65536

    wyL = bus.read_byte_data(address, 0x24)
    wyH = bus.read_byte_data(address, 0x25)
    wy = wyH * 256 + wyL
    if wy > 32767 :
        wy -= 65536
        
    wzL = bus.read_byte_data(address, 0x26)
    wzH = bus.read_byte_data(address, 0x27)
    wz = wzH * 256 + wzL
    if wz > 32767 :
        wz -= 65536

    axL = bus.read_byte_data(address, 0x28)
    axH = bus.read_byte_data(address, 0x29)
    ax = axH * 256 + axL
    if ax > 32767 :
        ax -= 65536
        
    ayL = bus.read_byte_data(address, 0x2A)
    ayH = bus.read_byte_data(address, 0x2B)
    ay = ayH * 256 + ayL
    if ay > 32767 :
        ay -= 65536
        
    azL = bus.read_byte_data(address, 0x2C)
    azH = bus.read_byte_data(address, 0x2D)
    az = azH * 256 + azL
    if az > 32767 :
        az -= 65536

    timestamp = time() - time_of_startup

    my_accels = array([ax, ay, az]) * ac_conv_factor
    my_gyros = array([wx, wy, wz]) * g_conv_factor
    my_vals = list(my_accels)
    my_vals.extend(list(my_gyros))
    my_vals.extend([timestamp])

    append_list_as_row(my_log, my_vals)

time_of_startup = time()    
append_list_as_row(my_log, ["Accel X", "Accel Y", "Accel Z", "Gyro X", "Gyro Y", "Gyro Z", "Timestamp"])

my_cond = 0
my_start = time()
print("START")
while True:  # my_cond<200:
    runOneIter(time_of_startup)
    #my_cond += 1
my_end = time()
print(my_end - my_start)

print("Success")
