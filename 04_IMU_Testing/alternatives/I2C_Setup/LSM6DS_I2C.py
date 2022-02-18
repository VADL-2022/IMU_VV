# -*- coding: utf-8 -*-
"""
Created on Thu Feb 17
@author: kdmen
YOU MUST DOWNLOAD SMSBUS ON THE RPI BEFORE YOU CAN RUN THIS:
"""

import smbus
import time


def zero_padding(tc, size):
    zp = "0"*(size - len(tc))
    return zp + tc
    

def tc2dec(tc, size):
    """
        ONLY WORKS FOR INTEGERS NOT FLOATING POINTS RN
        Also the negatives appear to be off by
        Also not sure if zeros get preserved or not on incoming tc's data

        This functions converts any size two's complement number to decimal
        Made for 16 bit
        
        tc is a STRING in two's complement
        size is the number of bits the integer is (e.g. length of the string)
    """
    
    binary = 0
    if tc[0]=="0":
        for i in range(size):
            binary += int(tc[size-i-1])*(2**i)
        return binary
    else:
        # Need to invert
        inverted_list = (["1" if digit=="0" else "0" for digit in tc])
        inverted_str = ""
        for i in inverted_list:
           inverted_str = inverted_str + i
        inverted_int = int(inverted_str)
        
        # Now need to add 1, LOGICAL ADDITION
        carry = 0
        temp = [0]*size
        temp[size-1] = 1
        for i in range(size):
            if carry == 0:
                if (inverted_str[size-i-1]==str(0)) and (temp[size-i-1]==0):
                    inverted_str[size-i-1]==str(0)
                elif ((inverted_str[size-i-1]==str(1)) and (temp[size-i-1]==0)) or ((inverted_str[size-i-1]==str(0)) and (temp[size-i-1]==1)):
                    inverted_str[size-i-1]==str(1)
                elif (inverted_str[size-i-1]==str(1)) and (temp[size-i-1]==1):
                    inverted_str[size-i-1]==str(0)
                    carry = 1
            elif carry == 1:
                if (inverted_str[size-i-1]==str(0)) and (temp[size-i-1]==0):
                    inverted_str[size-i-1]==str(1)
                    carry = 0
                elif ((inverted_str[size-i-1]==str(1)) and (temp[size-i-1]==0)) or ((inverted_str[size-i-1]==str(0)) and (temp[size-i-1]==1)):
                    inverted_str[size-i-1]==str(0)
                elif (inverted_str[size-i-1]==str(1)) and (temp[size-i-1]==1):
                    inverted_str[size-i-1]==str(1)
                    carry = 1
            else:
                print(carry)
                print("Carry is not 1 or 0: I'm broken")
                raise(ValueError)
            
        # Now calcualte the decimal value from the binary
        for i in range(size):
                binary += int(inverted_str[size-i-1])*(2**i)
                
        # Make the number negative
        binary *= -1
        
    return binary

# Get I2C bus
bus = smbus.SMBus(1)
output_size = 16  # Bits

# Address of IMU, with SA0 tied to VCC, is 1101011 or 6B

# CRTL1 Controlling Linear Acceleration Output, +/- 208 Hz
bus.write_byte_data(0x6B, 0x10, 0x57)
# CRTL2 Controlling Gyro +/- 245 dps
bus.write_byte_data(0x6B, 0x11, 0x50)

# READ IN DATA
wx = zero_padding(str(bus.read_byte_data(0x6B, 0x23)), output_size)
wy = zero_padding(str(bus.read_byte_data(0x6B, 0x25)), output_size)
wz = zero_padding(str(bus.read_byte_data(0x6B, 0x27)), output_size)
ax = zero_padding(str(bus.read_byte_data(0x6B, 0x29)), output_size)
ay = zero_padding(str(bus.read_byte_data(0x6B, 0x2B)), output_size)
az = zero_padding(str(bus.read_byte_data(0x6B, 0x2D)), output_size)

zero_padding(tc, size)

# Output data to screen
print("Acceleration in X-Axis : %d" %ax)
print("Acceleration in Y-Axis : %d" %ay)
print("Acceleration in Z-Axis : %d" %az)