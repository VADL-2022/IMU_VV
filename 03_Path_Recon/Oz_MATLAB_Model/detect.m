clear; clc; close all;

%% INPUTS
imu_data_file= '/Users/ozgurorun/Desktop/IMU_full_file.xlsx';

%% DETECT PARAMETERS
take_off_threshold_g = 50;
landing_threshold_g = 50;
landing_advance_time = 15;
predicted_flight_duration = 50; 

%% ALTITUDE PARAMETER
B = 6.5e-3; % temperature lapse rate in troposphere in K/m
R = 287; % ideal gas constant in J/(kg.K)
g = 9.80665; % grabity at sea level in m/s2
T_0 = 288.15; % standard air temperature in K
P_0 = 101.325; % standard air pressure in kPa

%% IMU PROCESS
imu_data = readcell(imu_data_file);
imu = cell2mat(imu_data(2:length(imu_data), [1 36 37 38 19])); 

%% EXTRACT TIME, ACCEL, AND ALTITUDE
imu_t = imu(:,1)- imu(1,1); imu_t = imu_t.';
imu_ax = imu(:,2); imu_ax = imu_ax.';
imu_ay = imu(:,3); imu_ay = imu_ay.';
imu_az = -1*imu(:,4); imu_az = imu_az.';
imu_a = sqrt(imu_ax.^2 + imu_ay.^2 + imu_az.^2);
imu_N = length(imu_t);

imu_temp = T_0*(imu(:,5)/P_0).^(R*B/g);
imu_alt = (T_0 - imu_temp)/B;

%% FIND TAKEOFF AND UPDATE THE ARRAYS
take_off_i = find(imu_a>take_off_threshold_g,1) - 3;

imu_t = imu_t(take_off_i:imu_N);
imu_t = imu_t - imu_t(1);

imu_ax = imu_ax(take_off_i:imu_N); 
imu_ay = imu_ay(take_off_i:imu_N);
imu_az = imu_az(take_off_i:imu_N);
imu_a = imu_a(take_off_i:imu_N);

imu_alt = imu_alt(take_off_i:imu_N);
imu_alt = imu_alt - imu_alt(1);

for i=1:length(imu_alt)
    if(imu_alt(i) <= 0)
        imu_alt(i) = 0;
    end
end

take_off_i = 1;

%% FIND LANDING AND UPDATE THE ARRAYS
[minDistance, minIndex] = min(abs(imu_t - (predicted_flight_duration-landing_advance_time)));
[maxDistance, maxIndex] = min(abs(imu_t - (predicted_flight_duration+landing_advance_time)));
temp_accel = imu_a(minIndex:maxIndex);
landing_i = find(temp_accel>landing_threshold_g,1)+minIndex;

imu_t = imu_t(1:landing_i);
imu_ax = imu_ax(1:landing_i); 
imu_ay = imu_ay(1:landing_i);
imu_az = imu_az(1:landing_i);
imu_a = imu_a(1:landing_i);
imu_alt = imu_alt(1:landing_i);

%% DISPLAY RESULTS
disp("Take-Off Time (s) = ");
disp(imu_t(take_off_i));
disp("Landing (s) = ");
disp(imu_t(landing_i));













