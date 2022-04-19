%% RAW PARAMETERS
B = 6.5e-3; % temperature lapse rate in troposphere in K/m
R = 287; % ideal gas constant in J/(kg.K)
g = 9.80665; % grabity at sea level in m/s2
T_0 = 288.15; % standard air temperature in K
P_0 = 101.325; % standard air pressure in kPa

%% IMPORT IMU DATA & PROCESS ALTITUDE
imu_data = readcell(imu_data_file);
imu = cell2mat(imu_data(33:end,[1 36 37 38 19]));

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





