clear; clc; close all;
imu_data_file= '/Users/ozgurorun/Desktop/IMU_SIM/Trunc_Fullscale1.xlsx';
imu_process();

%% RAW PARAMETERS
dt = 0.001;
t_1 = 17.51; % when we start modeling after drogue deployemeny
epsilon_w = 0.01; 
quad_model = true; 

%% DERIVED PARAMETERS
w_0 = -6; % apogee wind speed in m/s
K = -2; % guess K
tau2 = 3; % parachute opening time

w_0_mph = w_0/0.44704;
w_base = w_0_mph*(2/max(imu_alt))^(1/7);

%% FUNCTION CALLS
wind_density_profile();
model_1_after_drogue();

% find correct K until area under the curve = W_0
while abs(area_curve - w_0) > epsilon_w
    if abs(area_curve - w_0) > 1
        increment = 1;
    else
        if abs(area_curve - w_0) > 0.1
            increment = 0.1;
        else
            increment = 0.01;
        end
    end
    if area_curve > w_0
        K = K -increment;
    else
        K = K +increment;
    end
    model_1_after_drogue();
    disp("K is ")
    disp(K);
    disp("Area under the curve is ")
    disp(area_curve);
end

model_2_after_takeoff();

%% PLOTS
% X Acceleration
figure(3)
hold on
plot(imu_t, smoothdata(imu_ax, 'gaussian', 20),'linewidth', 1.2)
plot(t1+0.3, ax1,'linewidth', 1.2)
plot(t, ax,'linewidth', 1.2)
hold off

ylabel('X Acceleration (m/s2)','interpreter','latex', 'FontSize', 16);
xlabel('Time (s)','interpreter','latex', 'FontSize', 16);
title('X Acceleration vs Time',...
    'interpreter','latex', 'FontSize', 16);
legend('IMU Raw Data', 'Model 1 (after drogue)', 'Model 2 (after take-off)', ...
    'interpreter','latex','location','northeast','FontSize', 14);
grid on;
xlim([17 21])


% Altitude
figure(4)
hold on
plot(imu_t, imu_alt,'linewidth', 1.2)
plot(t(1:i_sim_end), z(1:i_sim_end),'linewidth', 1.2)
hold off

ylabel('Altitude (m)','interpreter','latex', 'FontSize', 16);
xlabel('Time (s)','interpreter','latex', 'FontSize', 16);
title('Model 2 (after take-off): Altitude vs Time',...
    'interpreter','latex', 'FontSize', 16);
legend('IMU Pressure Altitude', 'Simulation Altitude', 'interpreter',...
    'latex','location','northeast','FontSize', 14);
grid on;

% X Displacement
figure(1)
hold on
plot(t1, x1,'linewidth', 1.2)
plot(t, x,'linewidth', 1.2)
hold off

ylabel('X Displacement (m)','interpreter','latex', 'FontSize', 16);
xlabel('Time (s)','interpreter','latex', 'FontSize', 16);
title('X Displacement vs Time',...
    'interpreter','latex', 'FontSize', 16);
legend('Model 1 (after drogue)', 'Model 2 (after take-off)', ...
    'interpreter','latex','location','northeast','FontSize', 14);
grid on;


% X Velocity
figure(2)
hold on
plot(t1, vx1,'linewidth', 1.2)
plot(t, vx,'linewidth', 1.2)
%plot(imu_t, wind_profile_x,'linewidth', 1.2)
hold off

ylabel('X Velocity (m/s)','interpreter','latex', 'FontSize', 16);
xlabel('Time (s)','interpreter','latex', 'FontSize', 16);
title('X Velocity vs Time',...
    'interpreter','latex', 'FontSize', 16);
legend('Model 1 (after drogue)', 'Model 2 (after takeoff)', ...
    'interpreter','latex','location','northeast','FontSize', 14);
grid on;


