clear; clc; close all;
%% LAUNCH RAIL GPS INPUT (only change this part befor launch)
input_longitude = 34.8968490;%34.8968710;
input_latitude = 86.6202393;%86.6201964; % NEGLECT NEGATIVE SIGN

%% DETERMINE LAUNCH GRID
n = 20;
index_matrix = (0:n-1) + n*(0:n-1).';

longitude_start = 34.900091;
latitude_start = 86.624884;

longitude_end = 34.886289;
latitude_end = 86.608062;


longitude_interval = abs(longitude_start-longitude_end)/n;
latitude_interval = abs(latitude_start-latitude_end)/n;

longitude_matrix = zeros(n,n);
latitude_matrix = zeros(n,n);

temp_long = longitude_start;

for i=1:n
    for j=1:n
        longitude_matrix(i,j) = temp_long;
        latitude_matrix(i,j) = latitude_start - (j-1)*latitude_interval;
    end
    temp_long = temp_long - longitude_interval;
end

launch_grid = -1;

for i=1:n
    for j=1:n
        if ((longitude_matrix(i,j) >= input_longitude) && ...
                (input_longitude >= (longitude_matrix(i,j) - ...
                longitude_interval)))...
                && ((latitude_matrix(i,j) >= input_latitude) && ...
                (input_latitude >= (latitude_matrix(i,j) - ...
                latitude_interval)))
            
            launch_grid = index_matrix(i,j);
            
        end
    end
end

disp("Launch Grid: " + launch_grid);

%% DISPLACEMENT OUTPUT FROM IMU CODE (these values will come from imu code)
max_x = 500; 
min_x = -200;

max_y = -10;
min_y = 500;

average_x = (max_x+min_x)/2; 
average_y = (max_y+min_y)/2; 

%% DETERMINE LANDING GRIDS

grid_dimension = 1524;
long_to_gps = abs(longitude_start-longitude_end)/grid_dimension; 
lat_to_gps = abs(latitude_start-latitude_end)/grid_dimension; 

average_x_gps = long_to_gps*average_x + input_longitude; 
average_y_gps = lat_to_gps*average_y + input_latitude; 

max_x_gps = long_to_gps*max_x + input_longitude; 
max_y_gps = lat_to_gps*max_y + input_latitude; 

min_x_gps = long_to_gps*min_x + input_longitude; 
min_y_gps = lat_to_gps*min_y + input_latitude; 

average_landing_grid = -1;
max_landing_grid = -1;
min_landing_grid = -1;

for i=1:n
    for j=1:n
        if ((longitude_matrix(i,j) >= average_x_gps) && ...
                (average_x_gps >= (longitude_matrix(i,j) - ...
                longitude_interval)))...
                && ((latitude_matrix(i,j) >= average_y_gps) && ...
                (average_y_gps >= (latitude_matrix(i,j) - ...
                latitude_interval)))
            
            average_landing_grid = index_matrix(i,j);
            
        end
    end
end

for i=1:n
    for j=1:n
        if ((longitude_matrix(i,j) >= max_x_gps) && ...
                (max_x_gps >= (longitude_matrix(i,j) - ...
                longitude_interval)))...
                && ((latitude_matrix(i,j) >= max_y_gps) && ...
                (max_y_gps >= (latitude_matrix(i,j) - ...
                latitude_interval)))
            
            max_landing_grid = index_matrix(i,j);
            
        end
    end
end

for i=1:n
    for j=1:n
        if ((longitude_matrix(i,j) >= min_x_gps) && ...
                (min_x_gps >= (longitude_matrix(i,j) - ...
                longitude_interval)))...
                && ((latitude_matrix(i,j) >= min_y_gps) && ...
                (min_y_gps >= (latitude_matrix(i,j) - ...
                latitude_interval)))
            
            min_landing_grid = index_matrix(i,j);
            
        end
    end
end


disp("Average Landing Grid: " + average_landing_grid);
disp("Max Landing Grid: " + max_landing_grid);
disp("Min Landing Grid: " + min_landing_grid);









