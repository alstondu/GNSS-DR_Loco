% Kalman filter based GNSS solution with outlier detection
close all;
clear;
clc;
format long g;
Define_Constants;

% Load data
load('LS_init.mat','LS_init');
data_ws1_pseudo_ranges = readmatrix('Data\Pseudo_ranges.csv');
data_ws1_pseudo_range_rates = readmatrix('Data\Pseudo_range_rates.csv');
satellite_list = data_ws1_pseudo_ranges(1, 2:end); % [5,6,7,9,10,11,15,30]
time_list = data_ws1_pseudo_ranges(2:end, 1); % [0:0.5:425]
[dust, sat_num] = size(satellite_list); % sat_num=8
[time_num, dust] = size(time_list); % time_num=851

% constants
tau_s = 0.5; % (s), propagation interval
S_ae = 0.01; % (m^2s^-3), acceleration PSD
S_cphia = 0.01; % (m^2s^-3), clock phase PSD
S_cfa = 0.04; % (m^2s^-3), clock frequency PSD
sigma_pho = 10; % (m), range measurements standard deviation error
sigma_r = 0.05; % (m/s), range rate measurements standard deviation error

sigma_rho = 5; % (m) measurement standard deviation error for outlier detection
T = 6; % outlier detection threshold

GNSS = zeros(time_num, 7); % [time, latitude, longitude, height, vel_n, vel_e, vel_d] * 851
GNSS(1, 2) = LS_init(4);
GNSS(1, 3) = LS_init(5);
GNSS(1, 4) = LS_init(6);
GNSS(1, 5:7) = LS_init(7:9)';

pos_offset = LS_init(13); % position receiver clock offset
vel_offset = LS_init(14); % velocity receiver clock offset

% a) initialise the state vector and covariance matrix
x_pre_plus = [LS_init(1:3)'; LS_init(7:9)'; pos_offset; vel_offset];
P_pre_plus = [100 * eye(3), zeros(3,5);
              zeros(3,3), 0.01 * eye(3), zeros(3,2);
              zeros(1,6), 100, zeros(1,1);
              zeros(1,7), 0.01];

% b) compute the transition matrix
trans_mat = [eye(3), tau_s * eye(3), zeros(3,1), zeros(3,1);
             zeros(3,3), eye(3), zeros(3,1), zeros(3,1);
             zeros(1,3), zeros(1,3), 1, tau_s;
             zeros(1,3), zeros(1,3), 0, 1];

% c) compute the system noise covariance matrix
noise_mat = [1/3 * S_ae * tau_s^3 * eye(3), 1/2 * S_ae * tau_s^2 * eye(3), zeros(3,1), zeros(3,1);
             1/2 * S_ae * tau_s^2 * eye(3), S_ae * tau_s * eye(3), zeros(3,1), zeros(3,1);
             zeros(1,3), zeros(1,3), S_cphia * tau_s + 1/3 * S_cfa * tau_s^3, 1/2 * S_cfa * tau_s^2;
             zeros(1,3), zeros(1,3), 1/2 * S_cfa * tau_s^2, S_cfa * tau_s];

for iter = 2:time_num
    % d) use the transition matrix to propagate the state estimates
    x_cur_minus = trans_mat * x_pre_plus;

    % e) use this to propagate the error covariance matrix
    P_cur_minus = trans_mat * P_pre_plus * trans_mat' + noise_mat;

    % outlier detection
    is_outlier = 1;
    outlier_num = 0;
    residual_pre = 0;

    pos_j = zeros(3, sat_num); % Cartesian ECEF position of satellites at current time
    vel_j = zeros(3, sat_num); % Cartesian ECEF velocity of satellites at current time
    for j = 1:sat_num
        [pos_j(:,j), vel_j(:,j)] = Satellite_position_and_velocity(time_list(iter), satellite_list(j));
    end
    
    rho_a_j = data_ws1_pseudo_ranges(iter+1, 2:end); % measured range from satellites to the user antenna
    rho_dot_a_j = data_ws1_pseudo_range_rates(iter+1, 2:end); % measured range rates from satellites to the user antenna

    while is_outlier == 1 % loop until no outlier satellite is detected
        is_outlier = 0;

        % f) predict the ranges from the approximate user position to each satellite
        C = [1, 0, 0; 0, 1, 0; 0, 0, 1];
        C_eI = zeros(sat_num-outlier_num, 3, 3);

        r_aj_minus = zeros(1,sat_num-outlier_num);
        for j = 1:sat_num-outlier_num
            r_aj_minus(j) = sqrt((C * pos_j(:,j) - x_cur_minus(1:3))' * (C * pos_j(:,j) - x_cur_minus(1:3)));
            C_eI(j, :, :) = [1, omega_ie * r_aj_minus(j) / c, 0;
                            -omega_ie * r_aj_minus(j) / c, 1, 0;
                            0, 0, 1];
            r_aj_minus(j) = sqrt((squeeze(C_eI(j,:,:)) * pos_j(:,j) - x_cur_minus(1:3))' * (squeeze(C_eI(j,:,:)) * pos_j(:,j) - x_cur_minus(1:3)));
        end

        % g) compute the line-of-sight unit vector
        u_aj_e = zeros(3, sat_num-outlier_num);
        for j = 1:sat_num-outlier_num
            u_aj_e(:, j) = (squeeze(C_eI(j,:,:)) * pos_j(:,j) - x_cur_minus(1:3)) / r_aj_minus(j);
        end

        % h) predict the range rates
        r_aj_minus_dot = zeros(sat_num-outlier_num);
        for j = 1:sat_num-outlier_num
            r_aj_minus_dot(j) = u_aj_e(:,j)' * (squeeze(C_eI(j,:,:)) * (vel_j(:,j) + Omega_ie * pos_j(:,j)) - (x_cur_minus(4:6) + Omega_ie * x_cur_minus(1:3)));
        end

        % i) compute the measurement matrix
        H_cur = zeros((sat_num-outlier_num) * 2, 8);
        for j = 1:sat_num-outlier_num
            H_cur(j,1:3) = -u_aj_e(:, j)';
            H_cur(j+sat_num-outlier_num, 4:6) = -u_aj_e(:,j)';
            H_cur(j,7) = 1;
            H_cur(j+sat_num-outlier_num,8) = 1;
        end

        % j) compute the measurement noise covariance matrix
        R_cur = [sigma_pho^2 * eye(sat_num-outlier_num), zeros(sat_num-outlier_num, sat_num-outlier_num);
                zeros(sat_num-outlier_num, sat_num-outlier_num), sigma_r^2 * eye(sat_num-outlier_num)];

        % k) compute the kalman gain matrix
        kalman_gain = P_cur_minus * H_cur' * inv(H_cur * P_cur_minus * H_cur' + R_cur);

        % l) formulate the measurement innovation vector
        delta_z_minus = zeros((sat_num-outlier_num) * 2, 1);
        for j=1:sat_num-outlier_num
            delta_z_minus(j) = rho_a_j(j) - r_aj_minus(j) - x_cur_minus(7);
            delta_z_minus(j+sat_num-outlier_num) = rho_dot_a_j(j) - r_aj_minus_dot(j) - x_cur_minus(8);
        end
        
        % outlier detection part
        % compute the residuals vector
        v = (H_cur * inv(H_cur' * H_cur) * H_cur' - eye((sat_num-outlier_num) * 2)) * delta_z_minus;
        % compute the residuals covariance matrix
        C_v = (eye((sat_num-outlier_num) * 2) - H_cur * inv(H_cur' * H_cur) * H_cur') * sigma_rho^2;
        for k = 1:length(v)
            residual = abs(v(k)) - sqrt(C_v(k,k))*T;
            % locate the time and satellite with largest residual
            if residual > residual_pre
                idx = k;
                residual_pre = residual;
                is_outlier = 1;
                disp(['outlier occurred to satellite ' num2str(satellite_list(idx)) ' at time ' num2str(time_list(iter))]);
                % pause(0.25);
            end
        end
        if is_outlier == 1 % remove the data of the outlier satellite
            outlier_num = outlier_num + 1;
            pos_j(:,idx) = [];
            vel_j(:,idx) = [];
            rho_a_j(idx) = [];
            rho_dot_a_j(idx) = [];
        end

    end % while is_outlieDR_resultr == 1

    % m) update the state estimate
    x_cur_plus = x_cur_minus + kalman_gain * delta_z_minus;

    % n) update the error covariance matrix
    P_cur_plus = (eye(8) - kalman_gain * H_cur) * P_cur_minus;

    % o) convert to NED
    [lat_rad, long_rad, height, vel] = pv_ECEF_to_NED(x_cur_plus(1:3), x_cur_plus(4:6));
    lat = rad_to_deg * lat_rad;
    long = rad_to_deg * long_rad;

    % prepare for next loop
    x_pre_plus = x_cur_plus;
    P_pre_plus = P_cur_plus;
    % store the result
    GNSS(iter, 1) = time_list(iter);
    GNSS(iter, 2) = lat;
    GNSS(iter, 3) = long;
    GNSS(iter, 4) = height;
    GNSS(iter, 5:7) = vel';
end

save('GNSS.mat','GNSS');

% Plot the result
figure;
quiver(GNSS(:,3), GNSS(:,2), GNSS(:,6), GNSS(:,5), 0.5);
xlabel('Longitude (deg)');
ylabel('Latitude (deg)');
title('GNSS Velocity Vector Field');
xlim([min(GNSS(:,3)) max(GNSS(:,3))]);
ylim([min(GNSS(:,2)) max(GNSS(:,2))]);
grid on;