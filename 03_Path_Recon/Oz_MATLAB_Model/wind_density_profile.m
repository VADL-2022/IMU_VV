
%% CONSTURT WIND PROFILE & AIR DENSITY FOR ENTIRE FLIGHT
z_0 = max(imu_alt); % apogee altitude in m

wind_profile_x = zeros(1,imu_N);
density_profile = zeros(1,imu_N);

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