import pandas as pd
import numpy as np
from math import acos, sin, cos, tan

def sind(x):
    return sin(np.deg2rad(x))

def cosd(x):
    return cos(np.deg2rad(x))

def tand(x):
    return tan(np.deg2rad(x))

def acosd(x):
    return np.rad2deg(acos(x))

## Accept the IMU data somehow

def convert_YPR_fixed_frame(alpha, beta, gamma):
    '''
    This function returns the YPR in the fixed world frame.
    Data collected from the Vector Nav is in the body frame and must be transformed
     in order to do the necessary math later
    '''
    
    # yaw
    R_alpha = np.array([[1, 0, 0],
             [0, cosd(alpha), -sind(alpha)],
             [0, sind(alpha), cosd(alpha)]])

    # pitch
    R_beta = np.array([[cosd(beta), 0, sind(beta)],
            [0, 1, 0],
            [-sind(beta), 0, cosd(beta)]])

    # roll
    R_gamma = np.array([[cosd(gamma), -sind(gamma), 0],
             [sind(gamma), cosd(gamma), 0],
             [0, 0, 1]])
    
    R = R_gamma*R_beta*R_alpha

    # calculate angle
    angle = acosd((np.trace(R) - 1)/2)

    # theta
    theta = acosd(np.dot(R[:,2], [0, 0, 1]))

    return R #, angle, theta
