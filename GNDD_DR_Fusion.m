% Integrated DR/GNSS solution based on Kalman Filter
close all;
clear;
clc;
format long g;
Define_Constants;

% Load Data
load('GNSS.mat','GNSS');
load('DR.mat','DR');

% constants
S_DR = 0.01; % (m^2s^-3) DR velocity error power spectral density
sigma_Gr = 10;
sigma_Gv = 0.05;
sigma_v = 0.1; % (m/s) initial velocity uncertainty
sigma_r = 10; % (m) initial position uncertainty

L_pre = GNSS(1,2) * deg_to_rad;
[R_N, R_E] = Radii_of_curvature(L_pre);
h_pre = GNSS(1,4);
time_num = length(DR(:,1));
tau_s = 0.5;

% store integration result
Integrated_result = zeros(time_num, 6); % [time, latitude, longitude, speed_n, speed_e, heading]
Integrated_result(1,1) = 0;
Integrated_result(1,2) = GNSS(1,2);
Integrated_result(1,3) = GNSS(1,3);
Integrated_result(1,4) = GNSS(1,5);
Integrated_result(1,5) = GNSS(1,6);
Integrated_result(1,6) = DR(1,6);

% state vector
x_pre_plus = [0; 0; 0; 0];

% state estimation error covariance matrix
P_pre_plus = [sigma_v^2, 0, 0, 0;
              0, sigma_v^2, 0, 0;
              0, 0, sigma_r^2 / (R_N + h_pre)^2, 0;
              0, 0, 0, sigma_r^2 / ((R_E + h_pre)^2 * cos(L_pre)^2)];

for i = 2:time_num
    % compute the transition matrix
    trans_mat = [1, 0, 0, 0;
                 0, 1, 0, 0;
                 tau_s / (R_N + h_pre), 0, 1, 0;
                 0, tau_s / ((R_E + h_pre) * cos(L_pre)), 0, 1];

    % compute the system noise covariance matrix
    Q_pre = [S_DR * tau_s, 0, 1/2 * S_DR * tau_s^2 / (R_N + h_pre), 0;
             0, S_DR * tau_s, 0, 1/2 * S_DR * tau_s^2 / ((R_E + h_pre) * cos(L_pre));
              1/2 * S_DR * tau_s^2 / (R_N + h_pre), 0, 1/3 * S_DR * tau_s^3 / (R_N + h_pre)^2, 0;
              0, 1/2 * S_DR * tau_s^2 / ((R_E + h_pre) * cos(L_pre)), 0, 1/3 * S_DR * tau_s^3 / ((R_E + h_pre)^2 * cos(L_pre)^2)];

    % propagate the state estimates
    x_cur_minus = trans_mat * x_pre_plus;

    % propagate the error covariance matrix
    P_cur_minus = trans_mat * P_pre_plus * trans_mat' + Q_pre;

    % compute the measurement matrix
    H_cur = [0, 0, -1, 0;
             0, 0, 0, -1;
             -1, 0, 0, 0;
             0, -1, 0, 0];

    % compute the measurement noise covariance matrix
    L_cur = GNSS(i,2) * deg_to_rad;
    [R_N, R_E] = Radii_of_curvature(L_cur);
    h_cur = GNSS(i,4);

    R_cur = [sigma_Gr^2 / (R_N + h_cur)^2, 0, 0, 0;
             0, sigma_Gr^2 / ((R_E + h_cur)^2 * cos(L_cur)^2), 0, 0;
             0, 0, sigma_Gv^2, 0;
             0, 0, 0, sigma_Gv^2];

    % compute the Kalman gain matrix
    K_cur = P_cur_minus * H_cur' * inv(H_cur * P_cur_minus * H_cur' + R_cur);

    % formulate the measurement innovation vector
    delta_z_cur_minus = [(GNSS(i,2) - DR(i,2)) * deg_to_rad;
                         (GNSS(i,3) - DR(i,3)) * deg_to_rad;
                         GNSS(i,5) - DR(i,4);
                         GNSS(i,6) - DR(i,5)] - H_cur * x_cur_minus;

    % update the state estimates
    x_cur_plus = x_cur_minus + K_cur * delta_z_cur_minus;

    % update the error covariance matrix
    P_cur_plus = (eye(4) - K_cur * H_cur) * P_cur_minus;

    % use the Kalman filter estimates to correct the DR solution
    Integrated_result(i,1) = GNSS(i,1);
    Integrated_result(i,2) = (DR(i,2) * deg_to_rad - x_cur_plus(3)) * rad_to_deg;
    Integrated_result(i,3) = (DR(i,3) * deg_to_rad - x_cur_plus(4)) * rad_to_deg;
    Integrated_result(i,4) = DR(i,4) - x_cur_plus(1);
    Integrated_result(i,5) = DR(i,5) - x_cur_plus(2);
    Integrated_result(i,6) = DR(i,6);
    
    % prepare for next loop
    x_pre_plus = x_cur_plus;
    P_pre_plus = P_cur_plus;
    L_pre = L_cur;
    h_pre = h_cur;
end

% Plot the GNSS/DR result
figure;
quiver(Integrated_result(:,3), Integrated_result(:,2), Integrated_result(:,5), Integrated_result(:,4), 0.5);
xlabel('Longitude (deg)');
ylabel('Latitude (deg)');
title('GNSS/DR Velocity Vector Field');
xlim([min(Integrated_result(:,3)) max(Integrated_result(:,3))]);
ylim([min(Integrated_result(:,2)) max(Integrated_result(:,2))]);
grid on;

% Plot the 3 results
figure('Position', [100, 100, 800, 600]);
hold on;
quiver(GNSS(:,3), GNSS(:,2), GNSS(:,6), GNSS(:,5), 'Color', 'r', 'LineWidth', 0.25);
quiver(DR(:,3), DR(:,2), DR(:,5), DR(:,4), 'Color', 'g', 'LineWidth', 0.25);
quiver(Integrated_result(:,3), Integrated_result(:,2), Integrated_result(:,5), Integrated_result(:,4), 'Color', 'b', 'LineWidth', 1.5);
legend('GNSS', 'DR', 'GNSS/DR');
xlabel('Longitude (deg)');
ylabel('Latitude (deg)');
title('GNSS/DR Velocity Vector Field');
grid on;

% Save the result
fileID = fopen('Data\Integrated_result.csv', 'w');
fprintf(fileID, '%.1f,%.8f,%.8f,%.8f,%.8f,%.8f\n', Integrated_result.');
fclose(fileID);