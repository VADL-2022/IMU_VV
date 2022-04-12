## HELPER FUNCTIONS 
# Don't need to copy and paste these into every single NB
import numpy as np
from scipy import integrate
from math import sin, cos, pi, atan2, asin, acos

def sind(x):
    return sin(np.deg2rad(x))

def cosd(x):
    return cos(np.deg2rad(x))

def tand(x):
    return tan(np.deg2rad(x))

def acosd(x):
    return np.rad2deg(acos(x))

def nasa_pres(P, P0=101.29, T0=288.08):
    T = T0*((P / P0)**(1 / 5.256)) - 273.1
    h = (T-15.04)/(-0.00649)
    return h

def numerical_int(accel_xyz, tdata):
    '''
    This is jsut vanilla integration of whatever you give it, and integrates twice
    E.g. goes from acceleration to position
    
    accel_xyz is a 3x1 vector of all the accelerations in the XYZ for the rows
    '''
    
    ax_vn, ay_vn, az_vn = accel_xyz[0], accel_xyz[1], accel_xyz[2]

    vx_vn, vy_vn, vz_vn = np.zeros(len(ax_vn)),np.zeros(len(ay_vn)), np.zeros(len(az_vn))
    x_vn, y_vn, z_vn = np.zeros(len(ax_vn)),np.zeros(len(ay_vn)), np.zeros(len(az_vn))
    dt = tdata[1]

    vx_vn = integrate.cumtrapz(ax_vn, tdata, initial=0)
    vy_vn = integrate.cumtrapz(ay_vn, tdata, initial=0)
    vz_vn = integrate.cumtrapz(az_vn, tdata, initial=0)

    x_vn = integrate.cumtrapz(vx_vn, tdata, initial=0)
    y_vn = integrate.cumtrapz(vy_vn, tdata, initial=0)
    z_vn = integrate.cumtrapz(vz_vn, tdata, initial=0)
    
    v_vec = [vx_vn, vy_vn, vz_vn]
    x_vec = [x_vn, y_vn, z_vn]
    
    return v_vec, x_vec


def convert_ffYPR_matrix(alpha, beta, gamma):
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

    return R


def get_ffYPR_theta(bYPR):
    """Accepts body frame YPR, returns the FIXED FRAME YPR"""
    
    R = convert_ffYPR_matrix(bYPR[0], bYPR[1], bYPR[2])

    # Ensure that these are the correct graphs... eg matching ffpitch to bpitch...
    ffyaw = acosd(np.dot(R[:,1], [0, 1, 0]))
    ffpitch = acosd(np.dot(R[:,2], [0, 0, 1]))
    ffroll = acosd(np.dot(R[:,0], [1, 0, 0]))
    
    ADJ_ffpitch = ffpitch*-1 + 180
    #ADJ_ffroll = ffroll*-1 + 180 <-- USE THIS WHEN NOT USING ABG CONVERSION
    
    ffYPR = np.array([ffyaw, ADJ_ffpitch, ffroll])
    
    return ffYPR


def convert_abg(bYPR):
    # Normally, alpha beta gamma are yaw pitch roll
    # Our IMU placement is such that the roll axis of the IMU is the pitch of the vehicle
    # So let's just switch things around
    
    gamma = bYPR[0]  # Yaw
    alpha = bYPR[1]  # Pitch
    beta = bYPR[2]  # Roll

    # Essentially we just returned a switched around version of bYPR
    return np.array([alpha, beta, gamma])