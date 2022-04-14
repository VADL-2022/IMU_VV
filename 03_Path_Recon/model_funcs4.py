import numpy as np
from pandas import read_csv
from math import sin, cos, pi, atan2, asin, sqrt
import matplotlib.pyplot as plt
from scipy import integrate

def atan(ratio):
    # THIS IS IN RADIANS
    return atan2(ratio, 1)


def oz_ascent_model2(w_0, imu_alt, imu_t_flight, my_max_sim_time=90, my_theta=2*pi/180, my_ssm=3.2, my_dry_base=15.89):
    P_0 = 101.325
    T_0 = 288.15
    R = 287
    B = 6.5*10**-3
    g = 9.80665  # gravity at sea level in m/s2

    imu_N = len(imu_alt)

    ## CONSTANT WIND PROFILE & AIR DENSITY FOR ENTIRE FLIGHT
    z_0 = max(imu_alt)  # apogee altitude in m

    wind_profile_x = np.zeros(imu_N);
    density_profile = np.zeros(imu_N);

    for ii in range(imu_N):
        T = T_0 - B*imu_alt[ii];
        P = P_0*1000*(T/T_0)**(g/(R*B));
        density_profile[ii] = P/(R*T);
        if imu_alt[ii] < 2:
            wind_profile_x[ii] = w_0*((2/z_0)**(1/7));
        else:
            wind_profile_x[ii] = w_0*((imu_alt[ii]/z_0)**(1/7))

    pi = 3.1415
    
    ## LAUNCH DAY INPUTS
    ###########################################################
    theta_0 = my_theta  # launch angle array in radians
    SSM = my_ssm  # static stability margin
    m_dry_base = my_dry_base
    ###########################################################
    
    ## UPDATE THESE BEFORE FULLSCALE
    ###########################################################
    T_avg = 1000  # (1056) average motor thrust in N %change this based on apogee
    t_burn = 1.57  # motor burn time in s
    m_motor = 0.773  # motor propellant mass in kg
    ###########################################################
    
    m_dry = m_dry_base - m_motor  # rocket dry mass in kg
    number_of_time_steps = 2;

    ## MODEL #2: SIMULATE TRAJECTORY AFTER TAKE OFF
    max_sim_time = my_max_sim_time  # maximum simulation time in s
    dt = 0.001
    t = np.arange(0, max_sim_time, dt)  # time array
    N = len(t)  # time array size
    z = np.zeros(N)
    x = np.zeros(N)  # z and x displacement array
    vz = np.zeros(N)
    vx = np.zeros(N)  # z and x velocity array
    az = np.zeros(N)
    ax = np.zeros(N)  # z and x acceleration array
    v = np.zeros(N);
    m = np.zeros(N)  # mass array
    theta = np.zeros(N)  # angle array
    omega = np.zeros(N)  # angle array
    alpha = np.zeros(N)  # angle array

    # RAW PARAMETERS
    Cd = 0.39  # rocket drag coefficient
    Cd_side = 1  # rocket side drag coefficient
    L = 2.06  # rocket length in m
    D = 0.1524  # rocket diameter in m
    L_rail = 2  # launch rail transit in m

    # DERIVED PARAMETERS
    A_rocket = pi*(D**2)/4  # rocket cross sectional area in m2
    A_side_r = 0.3741  # rocket side area in m2
    m_wet = m_dry + m_motor  # rocket wet mass in kg
    m_dot = m_motor/t_burn  # motor burn rate in kg/s

    # SIMULATION PARAMETERS
    i = 1  # loop index
    z[i] = 0
    x[i] = 0  # initial displacement
    vz[i] = 0
    vx[i] = 0  # initial velocity
    m[i] = m_wet  # initial wet mass in kg
    ax[i] = T_avg/m[i]*sin(theta_0);
    az[i] = T_avg/m[i]*cos(theta_0) - g  # initial acceleration
    theta[i] = theta_0  # initial angle (launch) in radians
    i = i + 1  # increase loop

    ## STAGE 1: POWERED ASCENT ON LAUNCH RAIL
    # while z altitude is lower than the launch rail altitude
    while (np.linalg.norm((x[i-1], z[i-1])) < L_rail):

        theta[i] = theta_0  # constant angle until launch rail is cleared

        x[i] = x[i-1] + vx[i-1]*dt  # calculate x position
        z[i] = z[i-1] + vz[i-1]*dt  # calculate z position

        vz[i] = vz[i-1] + az[i-1]*dt  # calculate z velocity
        vx[i] = vx[i-1] + ax[i-1]*dt  # calculate x velocity
        v[i] = np.linalg.norm((vx[i], vz[i]))  # calculate velocity along axis

        m[i] = m[i-1] - m_dot*dt  # calculate mass

        ax[i] = T_avg/m[i]*sin(theta_0);
        az[i] = T_avg/m[i]*cos(theta_0) - g;

        i = i + 1  # increase simulation step

    t_LRE = t[i-1]  # launch rail exit time
    i_LRE = i -1;

    ## STAGE 2: WIND COCKING DURING POWERED ASCENT
    #[minDistance, imu_LRE] = min(abs(imu_alt - z[i_LRE]));
    minDistance = np.amin(abs(imu_alt - z[i_LRE]))
    imu_LRE = np.where(abs(imu_alt - z[i_LRE]) == minDistance)[0][0]

    w_LRE = abs(wind_profile_x[imu_LRE]);
    tau_ascent = (w_LRE/(T_avg/m[i_LRE]-g))*(SSM**2/(SSM-1));

    # while wind cocking occurs
    while (t[i-1] < t[i_LRE]+number_of_time_steps*tau_ascent):

        theta[i] = theta[i-1] + omega[i-1]*dt  # calculate angle
        omega[i] = omega[i-1] + alpha[i-1]*dt  # calculate angular velocity

        x[i] = x[i-1] + vx[i-1]*dt  # calculate x position
        z[i] = z[i-1] + vz[i-1]*dt  # calculate z position

        vz[i] = vz[i-1] + az[i-1]*dt  # calculate z velocity
        vx[i] = vx[i-1] + ax[i-1]*dt  # calculate x velocity
        v[i] = np.linalg.norm((vx[i], vz[i]))    

        #[minDistance, imu_index] = min(abs(imu_alt - z[i]));
        minDistance = np.amin(abs(imu_alt - z[i]))
        imu_index = np.where(abs(imu_alt - z[i]) == minDistance)[0][0]
        w = abs(wind_profile_x[imu_index])  # side wind calculation
        rho = density_profile[imu_index];

        m[i] = m[i-1] - m_dot*dt  # calculate mass

        I = 1/12*m[i]*(L**2)  # calculate inertia

        FD_side = 0.5*Cd_side*A_side_r*rho*((vx[i]+w)**2)  # calculate side drag
        FD = 0.5*Cd*rho*(v[i]**2)*A_rocket  # calculate drag along axis

        # calculate acceleration along rocket axis
        dv = (((T_avg-FD-FD_side*sin(theta[i]))/m[i])-g*cos(theta[i]))*dt;
        v[i] = v[i-1] + dv;

        vx[i] = v[i]*sin(theta[i]);
        vz[i] = v[i]*cos(theta[i]);

        # accelerations
        ax[i] = (dv/dt)*sin(theta[i]);
        az[i] = (dv/dt)*cos(theta[i]);
        alpha[i] = FD_side*SSM*D*cos(theta[i])/I;

        i = i + 1  # increase simulation step

    ## STAGE 3: POWERED ASCENT
    # while MECO is not reached
    while t[i-1] < t_burn:

        x[i] = x[i-1] + vx[i-1]*dt  # calculate x position
        z[i] = z[i-1] + vz[i-1]*dt  # calculate z position

        #[minDistance, imu_index] = min(abs(imu_alt - z[i]));
        minDistance = np.amin(abs(imu_alt - z[i]))
        imu_index = np.where(abs(imu_alt - z[i]) == minDistance)[0][0]
        rho = density_profile[imu_index];

        vz[i] = vz[i-1] + az[i-1]*dt  # calculate z velocity
        vx[i] = vx[i-1] + ax[i-1]*dt  # calculate x velocity
        v = sqrt((vz[i])**2 + (vx[i])**2)  # calculate velocity along axis

        theta[i] = atan(vx[i]/vz[i])  # calculate angle

        FD = 0.5*Cd*rho*(v**2)*A_rocket  # calculate drag along axis

        m[i] = m[i-1] - m_dot*dt  # calculate mass

        ax[i] = (T_avg-FD)*sin(theta[i])/m[i]  # calculate x accel.
        az[i] = (T_avg-FD)*cos(theta[i])/m[i]-g  # calculate y accel.

        i = i + 1  # increase simulation step

    ## STAGE 4: COAST ASCENT
    while (vz[i-1] > 0):
        x[i] = x[i-1] + vx[i-1]*dt  # calculate x position
        z[i] = z[i-1] + vz[i-1]*dt  # calculate z position

        #[minDistance, imu_index_111] = min(abs(imu_alt - z[i]));
        minDistance = np.amin(abs(imu_alt - z[i]))
        imu_index_111 = np.where(abs(imu_alt - z[i]) == minDistance)[0][0]
        rho = density_profile[imu_index_111];

        vz[i] = vz[i-1] + az[i-1]*dt  # calculate z velocity
        vx[i] = vx[i-1] + ax[i-1]*dt  # calculate x velocity
        v = sqrt((vz[i])**2 + (vx[i])**2)  # calculate velocity along axis

        theta[i] = atan(vx[i]/vz[i])  # calculate angle

        FD = 0.5*Cd*rho*(v**2)*A_rocket  #calculate drag along axis

        ax[i] = -FD*sin(theta[i])/m_dry  # calculate x accel.
        az[i] = -FD*cos(theta[i])/m_dry-g  # calculate y accel.

        i = i + 1  # increase simulation step
        
    x = [val for val in x if val!=0]
    if len(x)==0:
        x = [0]
    
    return x