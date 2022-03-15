clear; clc; close all;

%% RAW PARAMETERS
dt  = 0.001;
B = 6.5e-3; % temperature lapse rate in troposphere in K/m
R = 287; % ideal gas constant in J/(kg.K)
g = 9.80665; % grabity at sea level in m/s2
T_0 = 288.15; % standard air temperature in K
P_0 = 101.325; % standard air pressure in kPa
imu_end_time = 19; % visually determined
a_0 = 1; % visually determined
w_0 = -8.2; % visually determined

%% IMPORT IMU DATA & PROCESS ALTITUDE
%imu_data_file= '/Users/ozgurorun/Desktop/IMU_SIM/Trunc_Fullscale1.xlsx';
imu_data = readcell(imu_data_file);
imu = cell2mat(imu_data(32:end,[1 36 37 38 19]));

imu_t = imu(:,1)- imu(1,1); imu_t = imu_t.';
imu_N = length(imu_t);

imu_ax = imu(:,2); imu_ax = imu_ax.';
imu_ay = imu(:,3); imu_ay = imu_ay.';
imu_az = -1*imu(:,4); imu_az = imu_az.';

imu_temp = T_0*(imu(:,5)/P_0).^(R*B/g);
imu_alt = (T_0 - imu_temp)/B;
imu_alt = imu_alt - imu_alt(imu_N);

for ii=1:imu_N
    if(imu_alt(ii) <= 0)
        imu_alt(ii) = 0;
    end
end

imu_alt = imu_alt.';

%% FIND VELOCITY AND DISPLACEMENT FOR IMU
imu_vx = zeros(1,imu_N);
imu_vy = zeros(1,imu_N);
imu_vz = zeros(1,imu_N);

imu_x = zeros(1,imu_N);
imu_y = zeros(1,imu_N);
imu_z = zeros(1,imu_N);

% Initialize velocity and position
imu_vz(1) = 0;
imu_vx(1) = 0;
imu_vy(1) = 0;

imu_z(1) = 0;
imu_x(1) = 0;
imu_y(1) = 0;

% Find velocity and position
for ii=1:imu_N-1
    
    imu_vz(ii+1) = imu_vz(ii) + imu_az(ii)*(imu_t(ii+1)-imu_t(ii));
    imu_z(ii+1) = imu_z(ii) + imu_vz(ii)*(imu_t(ii+1)-imu_t(ii));
    
    imu_vx(ii+1) = imu_vx(ii) + imu_ax(ii)*(imu_t(ii+1)-imu_t(ii));
    imu_x(ii+1) = imu_x(ii) + imu_vx(ii)*(imu_t(ii+1)-imu_t(ii));
    
    imu_vy(ii+1) = imu_vy(ii) + imu_ay(ii)*(imu_t(ii+1)-imu_t(ii));
    imu_y(ii+1) = imu_y(ii) + imu_vy(ii)*(imu_t(ii+1)-imu_t(ii));
    
end

%% CONSTURT WIND PROFILE & AIR DENSITY FOR ENTIRE FLIGHT
wind_profile_x = zeros(1,imu_N);
density_profile = zeros(1,imu_N);

z_0 = max(imu_alt);

w_0_mph = w_0/0.44704;

for ii=1:imu_N
    T = T_0-B* imu_alt(ii);
    P = P_0*1000*(T/T_0)^(g/(R*B));
    density_profile(ii) = P/(R*T);
    if imu_alt(ii) < 2
        wind_profile_x(ii) = w_0*((2/z_0)^(1/7));
    else
        wind_profile_x(ii) = w_0*((imu_alt(ii)/z_0)^(1/7));
    end
end

%% MODEL #1: SIMULATE TRAJECTORY AFTER DROGUE
% Initial parameters
[minDistance22, imu_end_ii] = min(abs(imu_t - imu_end_time));
sim_start_t = imu_end_time;
sim_end_t = imu_t(imu_N);

% Construct arrays
sim_t = (sim_start_t:dt:sim_end_t);
sim_N = length(sim_t);

sim_ax = zeros(1,sim_N);
sim_vx = zeros(1,sim_N);
sim_x = zeros(1,sim_N);

% Initialize acceleration, velocity, and displacement
sim_vx(1) = 0;
sim_ax(1) = -a_0/(w_0^2)*((w_0-sim_vx(1))^2);
sim_x(1) = imu_x(imu_end_ii);
dV = sim_ax(1)*dt;

sim_curr_alt = zeros(1,sim_N);
sim_curr_alt(1) = imu_alt(imu_end_ii);

% Find acceleration, velocity, and displacement
for ii=1:sim_N-1
    
    sim_curr_time = sim_t(ii+1);
    [minDistance2, imu_index] = min(abs(imu_t - sim_curr_time));
    sim_curr_alt(ii+1) = imu_alt(imu_index);
    
    sim_x(ii+1) = sim_x(ii) + sim_vx(ii)*dt + 0.5*dV*dt;
    
    sim_vx(ii+1) = sim_vx(ii) + dV;
    
    if wind_profile_x(imu_index) < sim_vx(ii+1)
        
        sim_ax(ii+1) = -a_0/(w_0^2)*((wind_profile_x(imu_index)-sim_vx(ii+1))^2);
    else
        sim_ax(ii+1) = a_0/(w_0^2)*((wind_profile_x(imu_index)-sim_vx(ii+1))^2);
    end
    
    dV = sim_ax(ii+1)*dt;
end

% Combine IMU ascent with the simulation
t1 = horzcat(imu_t(1:imu_end_ii),sim_t);
ax1 = horzcat(imu_ax(1:imu_end_ii) , sim_ax);
vx1 = horzcat(imu_vx(1:imu_end_ii) , sim_vx);
x1 = horzcat(imu_x(1:imu_end_ii) , sim_x);

%% MODEL #2: SIMULATE TRAJECTORY AFTER TAKE OFF
max_sim_time = imu_t(imu_N); % maximum simulation time in s
t = (0:dt:max_sim_time); % time array
N = size(t); % time array size
z = zeros(N); x = zeros(N); % z and x displacement array
vz = zeros(N); vx = zeros(N); % z and x velocity array
az = zeros(N); ax = zeros(N); % z and x acceleration array
v = zeros(N);
m = zeros(N); % mass array
theta = zeros(N); % angle array
omega = zeros(N); % angle array
alpha = zeros(N); % angle array
theta_0 = 2*pi/180; % launch angle array in radians

% RAW PARAMETERS
m_dry = 16.57; % rocket dry mass in kg
Cd = 0.39; % rocket drag coefficient
Cd_side = 1; % rocket side drag coefficient
L = 2.06; % rocket length in m
D = 0.1524; % rocket diameter in m
SSM = 2.6; % static stability margin
T_avg = 1740; % average motor thrust in N
t_burn = 2.1; % motor burn time in s
m_motor = 1.76; % motor wet mass in kg
L_rail = 2; % launch rail transit in m
number_of_time_steps = 2;

% DERIVED PARAMETERS
A_rocket = pi*(D^2)/4; % rocket cross sectional area in m2
A_side_r = 0.374; % rocket side area in m2
m_wet = m_dry + m_motor; % rocket wet mass in kg
m_dot = m_motor/t_burn; % motor burn rate in kg/s

% SIMULATION PARAMETERS
i = 1; % loop index
z(i) = 0; x(i) = 0; % initial displacement
vz(i) = 0; vx(i) = 0; % initial velocity
ax(i) = 0; az(i) = 0; % initial acceleration
m(i) = m_wet; % initial wet mass in kg
theta(i) = theta_0; % initial angle (launch) in radians
i = i + 1; % increase loop

%% STAGE 1: POWERED ASCENT ON LAUNCH RAIL

% while z altitude is lower than the launch rail altitude
while (norm([x(i-1) z(i-1)]) < L_rail)

    theta(i) = theta_0; % constant angle until launch rail is cleared
    
    x(i) = x(i-1) + vx(i-1)*dt; % calculate x position
    z(i) = z(i-1) + vz(i-1)*dt; % calculate z position
    
    vz(i) = vz(i-1) + az(i-1)*dt; % calculate z velocity
    vx(i) = vx(i-1) + ax(i-1)*dt; % calculate x velocity
    v(i) = norm([vx(i) vz(i)]); % calculate velocity along axis
    
    m(i) = m(i-1) - m_dot*dt; % calculate mass
    
    ax(i) = T_avg/m(i)*sin(theta_0);
    az(i) = T_avg/m(i)*cos(theta_0) - g;
    
    i = i + 1; % increase simulation step
end

t_LRE = t(i-1); % launch rail exit time
i_LRE = i -1;

% STAGE 2: WIND COCKING DURING POWERED ASCENT
[minDistance, imu_LRE] = min(abs(imu_alt - z(i_LRE)));
w_LRE = abs(wind_profile_x(imu_LRE));
tau = (w_LRE/(T_avg/m(i_LRE)-g))*(SSM^2/(SSM-1));

% whole wind cocking occurs
while (t(i-1) < t(i_LRE)+number_of_time_steps*tau)
    
    theta(i) = theta(i-1) + omega(i-1)*dt; % calculate angle
    omega(i) = omega(i-1) + alpha(i-1)*dt; % calculate angular velocity
    
    x(i) = x(i-1) + vx(i-1)*dt; % calculate x position
    z(i) = z(i-1) + vz(i-1)*dt; % calculate z position
    
    vz(i) = vz(i-1) + az(i-1)*dt; % calculate z velocity
    vx(i) = vx(i-1) + ax(i-1)*dt; % calculate x velocity
    v(i) = norm([vx(i) vz(i)]); % calculate velocity along axis
    
  
    [minDistance, imu_index] = min(abs(imu_alt - z(i)));
    w = abs(wind_profile_x(imu_index)); % side wind calculation
    rho = density_profile(imu_index);
    
    m(i) = m(i-1) - m_dot*dt; % calculate mass
    
    I = 1/12*m(i)*(L^2); % calculate inertia
    
    FD_side = 0.5*Cd_side*A_side_r*rho*((vx(i)+w)^2); % calculate side drag
    FD = 0.5*Cd*rho*(v(i)^2)*A_rocket; % calculate drag along axis
    
    alpha = FD_side*D*cos(theta(i))*SSM/I; % calculate angular accel.
    
    % calculate acceleration along rocket axis
    dv = (((T_avg-FD-FD_side*sin(theta(i)))/m(i))-g*cos(theta(i)))*dt;
    v(i) = v(i-1) + dv;
    
    vx(i) = v(i)*sin(theta(i));
    vz(i) = v(i)*cos(theta(i));
    
    % accelerations
    ax(i) = (dv/dt)*sin(theta(i));
    az(i) = (dv/dt)*cos(theta(i));
    alpha(i) = FD_side*SSM*D*cos(theta(i))/I;
    
    i = i + 1; % increase simulation step
    
end

% STAGE 3: POWERED ASCENT

% while MECO is not reached
while t(i-1) < t_burn
    
    x(i) = x(i-1) + vx(i-1)*dt; % calculate x position
    z(i) = z(i-1) + vz(i-1)*dt; % calculate z position
    
    [minDistance, imu_index] = min(abs(imu_alt - z(i)));
    rho = density_profile(imu_index);
    
    vz(i) = vz(i-1) + az(i-1)*dt; % calculate z velocity
    vx(i) = vx(i-1) + ax(i-1)*dt; % calculate x velocity
    v = sqrt((vz(i))^2 + (vx(i))^2); % calculate velocity along axis
    
    theta(i) = atan(vx(i)/vz(i)); % calculate angle
    
    FD = 0.5*Cd*rho*(v^2)*A_rocket; % calculate drag along axis
    
    m(i) = m(i-1) - m_dot*dt; % calculate mass
    
    ax(i) = (T_avg-FD)*sin(theta(i))/m(i); % calculate x accel.
    az(i) = (T_avg-FD)*cos(theta(i))/m(i)-g; % calculate y accel.
    
    i = i + 1; % increase simulation step
    
end

% STAGE 4: COAST ASCENT
while (vz(i-1) > 0)
    
    x(i) = x(i-1) + vx(i-1)*dt; % calculate x position
    z(i) = z(i-1) + vz(i-1)*dt; % calculate z position
    
    [minDistance, imu_index_1] = min(abs(imu_alt - z(i)));
    rho = density_profile(imu_index_1);
    
    vz(i) = vz(i-1) + az(i-1)*dt; % calculate z velocity
    vx(i) = vx(i-1) + ax(i-1)*dt; % calculate x velocity
    v = sqrt((vz(i))^2 + (vx(i))^2); % calculate velocity along axis
    
    theta(i) = atan(vx(i)/vz(i)); % calculate angle
    
    FD = 0.5*Cd*rho*(v^2)*A_rocket;% calculate drag along axis
    
    ax(i) = -FD*sin(theta(i))/m_dry; % calculate x accel.
    az(i) = -FD*cos(theta(i))/m_dry-g; % calculate y accel.
    
    i = i + 1; % increase simulation step
end
t_end = i-1;

t_new = t(1:t_end); x_new = x(1:t_end);
z_new = z(1:t_end); ax_new = x(1:t_end);
az_new = az(1:t_end); vx_new = vx(1:t_end);
vz_new = vz(1:t_end); 

sim_new_t = sim_t - sim_t(1) + t_new(t_end);
sim_new_x = sim_x + abs(x_new(t_end)-sim_x(1));
t2 = horzcat(t_new,sim_new_t); 
x2 = horzcat(x_new,sim_new_x);

%% PLOT
figure(1)

hold on
plot(t1, x1,'linewidth', 1.2)
plot(t2, x2,'linewidth', 1.2)
hold off

ylabel('X Displacement (m)','interpreter','latex', 'FontSize', 16);
xlabel('Time (s)','interpreter','latex', 'FontSize', 16);
title('X Displacement vs Time',...
    'interpreter','latex', 'FontSize', 16);
legend('After Take-off', 'After Drogue', 'interpreter','latex',...
    'location','northeast','FontSize', 14);
grid on;

figure(2)

hold on
plot(imu_t, imu_alt,'linewidth', 1.2)
plot(t_new, z_new,'linewidth', 1.2)
hold off

ylabel('X Displacement (m)','interpreter','latex', 'FontSize', 16);
xlabel('Time (s)','interpreter','latex', 'FontSize', 16);
title('X Displacement vs Time',...
    'interpreter','latex', 'FontSize', 16);
legend('After Take-off', 'After Drogue', 'interpreter','latex',...
    'location','northeast','FontSize', 14);
grid on;

