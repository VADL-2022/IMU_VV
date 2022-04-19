
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
m(i) = m_wet; % initial wet mass in kg
ax(i) = T_avg/m(i)*sin(theta_0);
az(i) = T_avg/m(i)*cos(theta_0) - g; % initial acceleration
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
    
    [minDistance, imu_index_111] = min(abs(imu_alt - z(i)));
    rho = density_profile(imu_index_111);
    
    vz(i) = vz(i-1) + az(i-1)*dt; % calculate z velocity
    vx(i) = vx(i-1) + ax(i-1)*dt; % calculate x velocity
    v = sqrt((vz(i))^2 + (vx(i))^2); % calculate velocity along axis
    
    theta(i) = atan(vx(i)/vz(i)); % calculate angle
    
    FD = 0.5*Cd*rho*(v^2)*A_rocket;% calculate drag along axis
    
    ax(i) = -FD*sin(theta(i))/m_dry; % calculate x accel.
    az(i) = -FD*cos(theta(i))/m_dry-g; % calculate y accel.
    
    i = i + 1; % increase simulation step
end

i = i -1;
i_sim_end = i -1;


% Initialize acceleration, velocity, and displacement
ax(i_sim_end) = imu_ax(imu_end_ii);
dV = ax(i_sim_end)*dt;
t_drogue_2 = 0;
area_curve_2 = 0;
vx(i_sim_end) = 0;

% Find acceleration, velocity, and displacement
while t_drogue_2 <= tau2
    
    x(i) = x(i-1) + vx(i-1)*dt + 0.5*dV*dt;
    vx(i) = vx(i-1) + dV;
    
    if quad_model == true
        ax(i) = K*((t_drogue_2/tau2)^2)*...
            ((w_0-vx(i))^2) + ax(i_sim_end);
    else
        ax(i) = K*(1-exp(-t_drogue_2/tau2))*(1+1.2*exp(-t_drogue_2/tau2))*...
            ((w_0-vx(i))^2) + ax(i_sim_end);
    end
    
    dV = ax(i)*dt;
    
    t_drogue_2 = t_drogue_2 + dt;
    area_curve_2 = area_curve_2 + ax(i)*dt;
    
    i = i + 1;
end

while i-1 < length(t)
    [min1234, imu_index_1234] = min(abs(imu_t - t(i)));
    x(i) = x(i-1) + vx(i-1)*dt;
    vx(i) = wind_profile_x(imu_index_1234);
    %sim_ax(i) = (sim_vx(i)-sim_vx(i-1))/dt;
    i = i + 1;
end

t_end = i-1;

% Trim Arrays
t = t(1:t_end); x = x(1:t_end);
z = z(1:t_end); ax = ax(1:t_end);
az = az(1:t_end); vx = vx(1:t_end);
vz = vz(1:t_end); theta = theta(1:t_end);


