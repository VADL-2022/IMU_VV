import numpy as np
from pandas import read_csv
from math import sin, cos, pi, atan2, asin, sqrt
import matplotlib.pyplot as plt
from scipy import integrate

def nasa_pres(P, P0, T0, R, B, g):
        imu_temp = T0*(P/P0)**(R*B/g)
        imu_alt = (T0 - imu_temp)/B
        return imu_alt
    

def calc_moving_avg(axg21t, n, tdata21, dynamic_window=False, dynamic_n_timing=140, dynamic_n=80):        
    if dynamic_window:
        axg21s_1 = pd.Series(axg21t[0:dynamic_n_timing]).rolling(window=n).mean().iloc[n-1:].values
        axg21s_2 = pd.Series(axg21t).rolling(window=dynamic_n).mean().iloc[n-1:].values[dynamic_n_timing:]
        new_axg21s = list(axg21s_1) + list(axg21s_2)
    else:
        axg21s = pd.Series(axg21t).rolling(window=n).mean().iloc[n-1:].values
        new_axg21s = list(axg21s)
    while len(new_axg21s) < len(tdata21):
        new_axg21s = [0] + list(new_axg21s) + [0]
    return new_axg21s

def find_peak_and_wait(a_vec, t, imu_alt, thresh=50, post_drogue_delay=0.85, signal_length=3, time=1, t_sim_drogue=9.85, t_sim_landing=90):
    '''
    USE Z FOR DETECTION BUT THE SIGNAL WE WANT IS IN X AND Y
    '''
    axn, ayn, azn = np.array(a_vec[0], float), np.array(a_vec[1], float), np.array(a_vec[2], float)
        
    if 0: #time==0:
        above_threshold = abs(azn) > thresh
        takeoff_idx = list(above_threshold).index(True)

        # Get past the full takeoff spike
        post_takeoff_idx = takeoff_idx
        if azn[post_takeoff_idx] > 0:
            flip = -1
        else:
            flip = 1
        takeoff_offset = 0
        while (azn[post_takeoff_idx]*flip < 0):
            post_takeoff_idx += 1
        
        # NOW FIND DROGUE
        above_threshold = abs(azn[post_takeoff_idx:]) > thresh
        takeoff_idx = list(above_threshold).index(True)
        
        # Fix this for fullscale.  Subscale won't work cause there's not actual drogue spike
        ########################################################################
        # Now get past the transcience
        while t[past_takeoff_idx] < t[takeoff_idx+takeoff_offset]:
            past_takeoff_idx += 1

        end_drogue_idx = drogue_idx
        while t[end_drogue_idx] < (t[drogue_idx] + signal_length):
            end_drogue_idx += 1
            
        ax_1 = np.average(axn[drogue_idx:end_drogue_idx])
        ay_1 = np.average(ayn[drogue_idx:end_drogue_idx])
        a_1 = [ax_1, ay_1]
            
        # Find the landing based on time
        landing_idx = end_drogue_idx
        landing_time_advance = 15
        while t[landing_idx] < (t_sim_landing-landing_time_advance):
            landing_idx += 1
        while np.linalg.norm([axn[landing_idx], ayn[landing_idx], azn[landing_idx]]) < thresh and ((t[landing_idx] > t_sim_landing-landing_time_advance) and (t[landing_idx] < t_sim_landing*2)):
            landing_idx += 1

        return takeoff_idx-3, drogue_idx, end_drogue_idx, landing_idx, a_1
        ########################################################################
    elif time==1:
        # Detect takeoff
        takeoff_idx = np.argmax(abs(azn)>thresh)
        drogue_idx = takeoff_idx
        
        # Wait until sim_drogue
        t_takeoff = t[takeoff_idx]
        t_drogue = t_takeoff
        while t_drogue < (t_takeoff + t_sim_drogue):
            drogue_idx += 1
            t_drogue = t[drogue_idx]
            
        # Now wait until takeoff_delay passes
        t_signal = t_drogue
        end_drogue_idx = drogue_idx
        while t_signal < (t_drogue + post_drogue_delay):
            end_drogue_idx += 1
            t_signal = t[end_drogue_idx]
            
        # Get the resting acceleration
        ax_1 = np.average(axn[drogue_idx:end_drogue_idx])
        ay_1 = np.average(ayn[drogue_idx:end_drogue_idx])
        a_1 = [ax_1, ay_1]
        
        # Find the landing based on time
        landing_idx = end_drogue_idx
        landing_time_advance = 15
        
        '''
        t45_idx = next(x for x, val in enumerate(t) if val > (t[t_takeoff])+t_sim_landing-landing_time_advance)
        t65_idx = next(x for x, val in enumerate(t) if val > (t[t_takeoff])+t_sim_landing+landing_time_advance)
        
        temp_ax = axn[t45_idx:t65_idx]
        temp_ay = ayn[t45_idx:t65_idx]
        temp_az = azn[t45_idx:t65_idx]
        
        landing_idx = t45_idx
        while np.linalg.norm([temp_ax[landing_idx], temp_ay[landing_idx], temp_az[landing_idx]]) < thresh:
            landing_idx += 1
        '''
        
        '''
        while t[landing_idx] < (t_sim_landing-landing_time_advance):
            landing_idx += 1
        while np.linalg.norm([axn[landing_idx], ayn[landing_idx], azn[landing_idx]]) < thresh and ((t[landing_idx] > t_sim_landing-landing_time_advance) and (t[landing_idx] < t_sim_landing*2)):
            landing_idx += 1
        '''
        
        landing_idx = end_drogue_idx
        landing_time_advance = 15
        while t[landing_idx] < (t_sim_landing-0): #landing_time_advance
            landing_idx += 1
        #while imu_alt[landing_idx] > 25 and not_stuck_in_tree:
        #    landing_idx += 1
        
        return takeoff_idx-3, drogue_idx, end_drogue_idx, landing_idx, a_1
    else:
        print("BAD INPUT - FIX TIME PARAMETER")
        return -1, -1, -1, -1, -1

    
def atan(ratio):
    # THIS IS IN RADIANS
    return atan2(ratio, 1)


def calc_displacement2(datafile, zero_out=False, my_thresh=50, my_post_drogue_delay=1, my_signal_length=3, use_time=True, my_t_sim_drogue=9.85, my_t_sim_landing=50):
    '''
    1. Input the file, read the data in
    2. Use find_peak_and_wait to find the drogue peak and isolate the signal
    - Revamp this to use altitude instead of time?
    3. Plot altitude vs time
    4. 
    '''

    # Parameters
    dt = 0.001
    B = 6.5*10**-3   # temperature lapse rate in troposphere in K/m
    R = 287   # ideal gas constant in J/(kg.K)
    g = 9.80665  # gravity at sea level in m/s2
    T0 = 288.15   # standard air temperature in K
    P0 = 101.325   # standard air pressure in kPa
    pi = 3.1415
    ft = 3.2884  # ft/m
    ms2mph = 0.6818182*ft
    gs2mph = ms2mph * 9.8

    # Read in the dataframe
    fields = ['Timestamp', 'Pres',
    'Roll', 'Pitch', 'Yaw',
    'LinearAccelNed X', 'LinearAccelNed Y', 'LinearAccelNed Z']
    df = read_csv(datafile, skipinitialspace=True, usecols=fields)

    # Read Data Fields
    imu_t = df['Timestamp'].values
    dt = imu_t[1]
    imu_N = len(imu_t)
    imu_ax = df['LinearAccelNed X'].values
    imu_ay = df['LinearAccelNed Y'].values
    imu_az = df['LinearAccelNed Z'].values * -1
    imu_pres = df['Pres']

    ################## INIT VECTORS  ##################
    imu_vx, imu_vy, imu_vz, imu_x, imu_y, imu_z = (np.zeros((imu_N)), np.zeros((imu_N)), 
                                                   np.zeros((imu_N)), np.zeros((imu_N)), 
                                                   np.zeros((imu_N)), np.zeros((imu_N)))
    imu_vx_flight, imu_vy_flight, imu_vz_flight, imu_x_flight, imu_y_flight, imu_z_flight = (np.zeros((imu_N)), np.zeros((imu_N)), 
                                                   np.zeros((imu_N)), np.zeros((imu_N)), 
                                                   np.zeros((imu_N)), np.zeros((imu_N)))

    ################## Find alt  ##################
    vec_NASA_pres = np.vectorize(nasa_pres)
    imu_alt = vec_NASA_pres(imu_pres, P0, T0, R, B, g)
    imu_alt = [0] + list((imu_alt - np.mean(imu_alt[10:50]))[1:])
    imu_alt = [val if val > 0 else 0 for val in imu_alt]
    
    # Find drogue peak and calc wind velocity
    a_vec = [imu_ax, imu_ay, imu_az]
    takeoff_time, imu_start_time, imu_end_time, landing_idx, a_1 = find_peak_and_wait(a_vec, imu_t, imu_alt, thresh=my_thresh, post_drogue_delay=my_post_drogue_delay, signal_length=my_signal_length, time=use_time, t_sim_drogue=my_t_sim_drogue, t_sim_landing=my_t_sim_landing)
    # ^ RETURNED VALUES ARE ALL INDICES NOT ACTUAL TIMES
    ax_1, ay_1 = a_1[0], a_1[1]

    ## TRUNCATE ALL THE VECTORS
    # This is the truncated signal we're interested in
    imu_ax_flight = imu_ax[takeoff_time:landing_idx]
    imu_ay_flight = imu_ay[takeoff_time:landing_idx]
    imu_az_flight = imu_az[takeoff_time:landing_idx]
    imu_alt = imu_alt[takeoff_time:landing_idx]
    imu_t_flight = imu_t[takeoff_time:landing_idx]
    imu_t_flight = imu_t_flight - imu_t_flight[0]

    # Find the max altitude
    z_0 = max(imu_alt)
    apogee_idx = list(imu_alt).index(z_0)
    # find_peak_and_wait returns the "signal" start and finish times
    # Alternatively, we could also add 1 second to the apogee_idx and find the corresponding index
    temp = imu_t_flight - imu_t_flight[apogee_idx]  - my_post_drogue_delay
    masked_temp = np.array([val if abs(val)>10**-5 else 0 for val in temp])
    my_min = min(abs(masked_temp))
    if my_min < 10**-5:
        my_min = 0
    try:
        imu_start_time = list(masked_temp).index(my_min)
    except ValueError:
        imu_start_time = list(masked_temp).index(-my_min)

    temp = imu_t_flight - imu_t_flight[apogee_idx] - my_post_drogue_delay - my_signal_length
    masked_temp = np.array([val if abs(val)>10**-5 else 0 for val in temp])
    my_min = min(abs(masked_temp))
    if my_min < 10**-5:
        my_min = 0
    try:
        imu_end_time = list(masked_temp).index(my_min)
    except ValueError:
        imu_end_time = list(masked_temp).index(-my_min)

    dt = imu_t_flight[1]

    imu_ax_signal = imu_ax_flight[imu_start_time:imu_end_time]
    imu_ay_signal = imu_ay_flight[imu_start_time:imu_end_time]
    
    # Took these out because there is a huge dependence on dt, which is also not constant
    #w0x = integrate.trapz(imu_ax_signal, dx=dt)
    #w0y = integrate.trapz(imu_ay_signal, dx=dt)
    #print(f"SCIPY WIND SPEEDS, X->{w0x} m/s and Y->{w0y} m/s")

    # Find the displacement after imu_end_time
    ################## Find velocity and position  ##################
    for i in range(len(imu_t_flight)-1):
        imu_vz_flight[i+1] = imu_vz_flight[i] + imu_az_flight[i]*(imu_t_flight[i+1] - imu_t_flight[i])
        imu_z_flight[i+1] = imu_z_flight[i] + imu_vz_flight[i]*(imu_t_flight[i+1] - imu_t_flight[i])

        imu_vx_flight[i+1] = imu_vx_flight[i] + imu_ax_flight[i]*(imu_t_flight[i+1] - imu_t_flight[i])
        imu_x_flight[i+1] = imu_x_flight[i] + imu_vx_flight[i]*(imu_t_flight[i+1] - imu_t_flight[i])

        imu_vy_flight[i+1] = imu_vy_flight[i] + imu_ay_flight[i]*(imu_t_flight[i+1] - imu_t_flight[i])
        imu_y_flight[i+1] = imu_y_flight[i] + imu_vy_flight[i]*(imu_t_flight[i+1] - imu_t_flight[i])
        
    w0x = imu_vx_flight[imu_end_time]-imu_vx_flight[imu_start_time]
    w0y = imu_vy_flight[imu_end_time]-imu_vy_flight[imu_start_time]
    print(f"NUM INT WIND SPEEDS, X->{imu_vx_flight[imu_end_time]-imu_vx_flight[imu_start_time]} m/s and Y->{imu_vy_flight[imu_end_time]-imu_vy_flight[imu_start_time]} m/s")

    drogue_opening_displacement_x = imu_x_flight[imu_end_time] - imu_x_flight[imu_start_time]
    drogue_opening_displacement_y = imu_y_flight[imu_end_time] - imu_y_flight[imu_start_time]

    m1_final_x_displacements, m1_final_y_displacements = [0]*3, [0]*3
    m2_final_x_displacements, m2_final_y_displacements = [0]*3, [0]*3
    for idx, uncertainty in enumerate([-1, 0, 1]):
        #For end_time to landing
        total_x_displacement = 0
        total_y_displacement = 0
        for i in range(imu_end_time, landing_idx-takeoff_time):
            vx = (w0x+uncertainty)*((imu_alt[i]/z_0)**(1/7))
            vy = (w0y+uncertainty)*((imu_alt[i]/z_0)**(1/7))
            total_y_displacement += vy*(imu_t_flight[i] - imu_t_flight[i-1])
            total_x_displacement += vx*(imu_t_flight[i] - imu_t_flight[i-1])

        # Oz Ascent Model
        m1_final_x_displacements[idx] = (imu_x_flight[imu_start_time] - imu_x_flight[takeoff_time]) + drogue_opening_displacement_x + total_x_displacement
        m1_final_y_displacements[idx] = (imu_y_flight[imu_start_time] - imu_y_flight[takeoff_time]) + drogue_opening_displacement_y + total_y_displacement

        # Oz's Other Ascent Model (Model 2) In Place of Marissa's Model
        m2_final_x_displacements[idx] = PYFILE_oz_ascent_model2(abs(w0x+uncertainty), imu_alt, imu_t_flight, my_max_sim_time=50)[-1] + drogue_opening_displacement_x + total_x_displacement
        m2_final_y_displacements[idx] = PYFILE_oz_ascent_model2(abs(w0y+uncertainty), imu_alt, imu_t_flight, my_max_sim_time=50)[-1] + drogue_opening_displacement_y + total_y_displacement

        print(f"MODEL 1: TOTAL X AND Y DISPLACEMENTS, u={uncertainty}: X->{m1_final_x_displacements[idx]:2f} m, Y->{m1_final_y_displacements[idx]:2f} m")
        print(f"MODEL 2: TOTAL X AND Y DISPLACEMENTS, u={uncertainty}: X->{m2_final_x_displacements[idx]} m, Y->{m2_final_y_displacements[idx]} m")
        print()

        # Take max and min of ALL 6 --> Then average for final result
        all_xs = []
        all_xs.extend(m1_final_x_displacements)
        all_xs.extend(m2_final_x_displacements)

        all_ys = []
        all_ys.extend(m1_final_y_displacements)
        all_ys.extend(m2_final_y_displacements)

        minx = min(all_xs)
        maxx = max(all_xs)
        avg_x = (minx+maxx)/2

        miny = min(all_ys)
        maxy = max(all_ys)
        avg_y = (miny+maxy)/2

    #return [x1[landing_idx], x2[landing_idx]], [y1[landing_idx], y2[landing_idx]], [x1, x2, t1, t2, y1, y2, ty1, ty2], [takeoff_time, imu_start_time, imu_end_time, landing_idx]
    return avg_x, avg_y

def PYFILE_oz_ascent_model2(w_0, imu_alt, imu_t_flight, my_max_sim_time=90):
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
    theta_0 = 2*pi/180  # launch angle array in radians
    SSM = 3.2  # static stability margin
    T_avg = 1000  # (1056) average motor thrust in N %change this based on apogee
    t_burn = 1.57  # motor burn time in s
    m_motor = 0.773  # motor propellant mass in kg
    m_dry = 15.89 - m_motor  # rocket dry mass in kg
    number_of_time_steps = 2;

    ## MODEL #2: SIMULATE TRAJECTORY AFTER TAKE OFF
    max_sim_time = my_max_sim_time  # maximum simulation time in s
    #^ = imu_t_flight[-1]
    dt = imu_t_flight[1]
    #t = np.arange(0, max_sim_time, dt)  # time array
    t = imu_t_flight
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
    
    return x