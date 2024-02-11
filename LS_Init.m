%  This script initialises position and velocity at time=0 using
%  least-queare algorithm
close all;
clear;
clc;
format long g;
Define_Constants;

% read files
data_ws1_pseudo_ranges = readmatrix('Data/Pseudo_ranges.csv');
data_ws1_pseudo_range_rates = readmatrix('Data/Pseudo_range_rates.csv');

satellite_list = data_ws1_pseudo_ranges(1, 2:end); % [5,6,7,9,10,11,15,30]
time_list = data_ws1_pseudo_ranges(2:end, 1); % [0:0.5:425]
[dust, sat_num] = size(satellite_list); % sat_num=8
[time_num, dust] = size(time_list); % time_num=851

% Vairable initialisation
latitude_0 = 0; % (degree) longitude at time 0
longitude_0 = 0; % (degree) latitude at time 0
height_0 = 0; % (m) height at time 0 in NED
vel_0 = [0; 0; 0]; % (m/s) velocity at time 0 in NED

pos_offset = 0.0; % position receiver clock offset
vel_offset = 0.0; % velocity receiver clock offset

diff = 100;
epoch_time = 0;

while diff > 0.001 % threshold when terminate iteration
    % convert NED to ECEF
    [user_appro_pos, user_appro_vel] = pv_NED_to_ECEF(latitude_0*deg_to_rad, longitude_0*deg_to_rad, height_0, vel_0);

    % compute the ECEF positions and velocities of the satellites at time 0
    satellite_pos = zeros(3,sat_num);
    satellite_vel = zeros(3,sat_num);
    for i = 1:sat_num
        [satellite_pos(:,i), satellite_vel(:,i)] = Satellite_position_and_velocity(time_list(1), satellite_list(i));
    end

    % predict the ranges from the approximate user position to each satellite
    C = [1, 0, 0; 0, 1, 0; 0, 0, 1];
    C_I = zeros(sat_num, 3, 3);
    ranges_list = zeros(1,sat_num);
    for i = 1:sat_num
        % compute the range with the Sagnac effect compensation matrix set to the identity matrix
        ranges_list(i) = sqrt((C * satellite_pos(:,i) - user_appro_pos)' * (C * satellite_pos(:,i) - user_appro_pos));
        % compute the Sagnac effect compensation matrix
        C_I(i,:,:) = [1, omega_ie * ranges_list(i) / c, 0;
                        -omega_ie * ranges_list(i) / c, 1, 0;
                        0, 0, 1];
        % compute the ranges
        ranges_list(i) = sqrt((squeeze(C_I(i,:,:)) * satellite_pos(:,i) - user_appro_pos)' * (squeeze(C_I(i,:,:)) * satellite_pos(:,i) - user_appro_pos));
    end

    % compute the unit vector
    unit_vector = zeros(3,sat_num);
    for i = 1:sat_num
        unit_vector(:,i) = (squeeze(C_I(i,:,:)) * satellite_pos(:,i) - user_appro_pos) / ranges_list(i);
    end

    % predict the range rates from the approximate user velocity to each satellite
    range_rates_list = zeros(1,sat_num);
    for i = 1:sat_num
        range_rates_list(i) = unit_vector(:,i)' * ((squeeze(C_I(i,:,:)) * (satellite_vel(:,i) + Omega_ie * satellite_pos(:,i)) - (user_appro_vel + Omega_ie * user_appro_pos)));
    end

    % formulate the predicted state vector, measurement innovation vector and measurement matrix
    x_hat_minus = zeros(4,1); % predicted state vector for position
    x_hat_minus(1:3, 1) = user_appro_pos;
    x_hat_minus(4,1) = pos_offset;

    delta_z_minus = zeros(sat_num,1); % measurement innovation vector for position
    rho_a_j = data_ws1_pseudo_ranges(2, 2:end); % measured range from satellite to the user antenna
    for i = 1:sat_num
        delta_z_minus(i) = rho_a_j(i) - ranges_list(i) - pos_offset;
    end

    H = zeros(sat_num,4); % measurement matrix
    for i = 1:sat_num
        H(i, 1:3) = -unit_vector(:,i);
        H(i, 4) = 1;
    end

    x_dot_hat_minus = zeros(4,1); % predicted state vector for velocity
    x_dot_hat_minus(1:3, 1) = user_appro_vel;
    x_dot_hat_minus(4,1) = vel_offset;

    delta_z_dot_minus = zeros(sat_num,1); % measurement innovation vector for velocity
    rho_dot_a_j = data_ws1_pseudo_range_rates(2, 2:end); % measured range rates from satellite to the user antenna
    for i = 1:sat_num
        delta_z_dot_minus(i) = rho_dot_a_j(i) - range_rates_list(i) - vel_offset;
    end

    % f) calculate the results
    pos_cal = x_hat_minus + (H' * H) \ H' * delta_z_minus;
    vel_cal = x_dot_hat_minus + (H' * H) \H' * delta_z_dot_minus;

    % g) change to NED frame and update clock offset
    [lat_exact, lon_exact, height_exact, vel_exact] = pv_ECEF_to_NED(pos_cal(1:3), vel_cal(1:3));
    lat_exact = lat_exact * rad_to_deg;
    lon_exact = lon_exact * rad_to_deg;
    pos_offset = pos_cal(4);
    vel_offset = vel_cal(4);

    % h) prepare for next iteration
    diff = sqrt((lat_exact-latitude_0)^2 + (lon_exact-longitude_0)^2 + (height_exact-height_0)^2 + sum((vel_exact-vel_0).^2));
    latitude_0 = lat_exact;
    longitude_0 = lon_exact;
    height_0 = height_exact;
    vel_0 = vel_exact;
    epoch_time = epoch_time + 1;
end

% disp('t=0:');
% disp(latitude_0);
% disp(longitude_0);
% disp(height_0);
% disp(vel_0);
% disp(pos_offset);
% disp(vel_offset);
vel_0_NED = vel_0;

% final initial position and velocity for step 2 and step 4
[pos_0, vel_0] = pv_NED_to_ECEF(latitude_0*deg_to_rad, longitude_0*deg_to_rad, height_0, vel_0);
pos_0_NED = [latitude_0, longitude_0, height_0];
offsets = [pos_offset, vel_offset];
LS_init = [pos_0', pos_0_NED, vel_0', vel_0_NED', offsets];

save('LS_init.mat','LS_init');