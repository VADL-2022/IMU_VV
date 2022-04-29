import numpy as np
from math import sin, cos, pi, atan2, asin, acos
import matplotlib.pyplot as plt

def sind(x):
    return sin(np.deg2rad(x))

def cosd(x):
    return cos(np.deg2rad(x))

def tand(x):
    return tan(np.deg2rad(x))

def acosd(x):
    return np.rad2deg(acos(x))


def calc_ffangle_arrays(t_array, alpha_array, beta_array, gamma_array, t_apogee=17.21):
    N = len(t_array)
    
    # Preallocate
    angle_array = np.zeros(N)  # angle
    n_array = np.zeros((3,N))  # axis
    theta_array = np.zeros(N)  # resolved angle
    
    ## Calculate
    # iterate until apogee
    for i in range(N):

        alpha = alpha_array[i]
        beta = beta_array[i]
        gamma = gamma_array[i]

        R, angle, theta = convert_fftheta(alpha, beta, gamma)
        angle_array[i] = angle

        # theta
        #theta = acosd(np.dot(R[:,2], [0, 0, 1]))
        theta_array[i] = theta

    # Theta vs Time
    pitch_array = -1*theta_array
    pitch_array = pitch_array - pitch_array[0]
    t_pre_apog = t_array[t_array < t_apogee]
    i_apogee = t_pre_apog.shape[0]
    pitch_array_apog = pitch_array[0:i_apogee]

    # Pitch vs Time
    gamma_array_apog = gamma_array[0:i_apogee]
    
    return t_pre_apog, pitch_array_apog, gamma_array_apog


def make_pitch_plot(t_pre_apog, pitch_array_apog):
    # Theta vs. Time
    fig1 = plt.figure(1, figsize=(8,8))
    ax = plt.subplot(111)
    l1 = ax.plot(t_pre_apog, pitch_array_apog, color='black', label='SLI Flight')
    l2 = plt.axvline(x=2.5, color='blue', linestyle="--", label='Wind Shear (at about 1500 ft)')
    l3 = plt.axvline(x=17.5, color='red', linestyle="--", label='Apogee Reached')
    plt.grid()
    #plt.yticks(np.linspace(0,90, num=11))
    #plt.ylim((0, 90))
    #plt.xticks(np.linspace(0,18, num=10))
    ax.set_title("Pitch Angle vs. Time (Until Apogee)")
    ax.set_xlabel('Time (s)')
    ax.set_ylabel("Theta: Pitch Angle [deg]")
    ax.legend(loc=2)
    plt.show()
    

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


def convert_fftheta(alpha, beta, gamma):
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

    return R, angle, theta