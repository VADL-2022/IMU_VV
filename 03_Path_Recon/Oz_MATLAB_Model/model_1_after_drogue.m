
%% RAW PARAMETERS
imu_end_time = t_1; % uncertainty start time in s (visually determined)

%% MODEL #1: SIMULATE X TRAJECTORY AFTER DROGUE
% Initial parameters
[min111, imu_end_ii] = min(abs(imu_t - imu_end_time));
sim_end_t = imu_t(imu_N);

% Construct arrays
sim_t = (imu_end_time:dt:sim_end_t);
sim_N = length(sim_t);
sim_ax = zeros(1,sim_N);
sim_vx = zeros(1,sim_N);
sim_x = zeros(1,sim_N);
wind_sim_x = zeros(1,sim_N);

% Initialize acceleration, velocity, and displacement
sim_vx(1) = 0;
sim_ax(1) = imu_ax(imu_end_ii);
sim_x(1) = imu_x(imu_end_ii);
dV = sim_ax(1)*dt;
t_drogue = 0;
area_curve = 0;
i = 2;

% Find acceleration, velocity, and displacement
while t_drogue <= tau2
    
    sim_x(i) = sim_x(i-1) + sim_vx(i-1)*dt + 0.5*dV*dt;
    sim_vx(i) = sim_vx(i-1) + dV;
    
    if quad_model == true
        sim_ax(i) = K*((t_drogue/tau2)^2)*...
            ((w_0-sim_vx(i))^2) + sim_ax(1);
    else
        sim_ax(i) = K*(1-exp(-t_drogue/tau2))*...
            (1+1.2*exp(-t_drogue/tau2))*((w_0-sim_vx(i))^2) + sim_ax(1);
    end
    
    dV = sim_ax(i)*dt;
    
    t_drogue = t_drogue + dt;
    area_curve = area_curve + sim_ax(i)*dt;
    
    i = i + 1;
end

while i-1 < sim_N
    [min123, imu_index_123] = min(abs(imu_t - sim_t(i)));
    sim_x(i) = sim_x(i-1) + sim_vx(i-1)*dt;
    sim_vx(i) = wind_profile_x(imu_index_123);
    %sim_ax(i) = (sim_vx(i)-sim_vx(i-1))/dt;
    i = i + 1;
end


% Combine IMU ascent with Model 1 (after drogue) simulation
t1 = horzcat(imu_t(1:imu_end_ii), sim_t);
ax1 = horzcat(imu_ax(1:imu_end_ii), sim_ax);
vx1 = horzcat(imu_vx(1:imu_end_ii), sim_vx);
x1 = horzcat(imu_x(1:imu_end_ii), sim_x);

