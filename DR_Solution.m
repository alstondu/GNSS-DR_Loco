% DR Solution
% Assumption: The antenna is at the center of rotation
close all;
clear
clc
format long g;
Define_Constants;

%% Constants
L_k0 = deg2rad(51.5092544612951); % Initial Latitude
Lamda_k0 = deg2rad(-0.161045484926378); % Initial Longitude
delta_t = 0.5; % Time interval
d_lf = 0.4; % Distance between left and right wheel
d_ar = 0.05; % Distance between center of mass and the antenna

%% Extract data from file
% Load the heading derived from integrating the gyroscope and the compass
load('LS_init.mat','LS_init');
load('Integrated_Heading.mat','Psi_gyro_c'); 
load('GNSS.mat','GNSS');
Psi_gyro_c = deg2rad(Psi_gyro_c); % Gyroscope heading corrected by magnetic compass
h = GNSS(:,4);

Dead_reckoning = readmatrix("Data/Dead_reckoning.csv");
t = Dead_reckoning(:,1); % time

v_f_a = (Dead_reckoning(:, 4) + Dead_reckoning(:, 5))/2; % Average speed of driving wheels
yaw_rate = (Dead_reckoning(:, 4) - Dead_reckoning(:, 5))/d_lf; % Yaw rate
v_ant_yaw = yaw_rate * d_ar; % Lateral velocity at the antenna due to yaw
v_ant_a = sqrt(v_f_a.^2 + v_ant_yaw.^2); % Average speed at the antenna

%% Initialization
v_a = zeros(2,1); % NED speed
L_k = zeros(length(t),1); % Latitude
L_k(1) = L_k0;
Lamda_k = zeros(length(t),1); % Longitude
Lamda_k(1) = Lamda_k0;
v_i = zeros(length(t),2); % Instantaneous velocity
v_i(1,1) = LS_init(10);
v_i(1,2) = LS_init(11);

%% DR Iteration
for i = 2:length(t)
    V_a2NED = [cos(Psi_gyro_c(i)) + cos(Psi_gyro_c(i-1));
                sin(Psi_gyro_c(i)) + sin(Psi_gyro_c(i-1))];
    % Average speed
    v_a = 0.5*V_a2NED*v_ant_a(i);
    [R_N,R_E] = Radii_of_curvature(L_k(i-1));
    % latitude
    L_k(i) = L_k(i-1) + v_a(1)*delta_t/(R_N + h(i-1));
    % longitude
    Lamda_k(i) = Lamda_k(i-1) + v_a(2)*delta_t/((R_E + h(i-1))*cos(L_k(i)));
    % Instantaneous speed
    v_i(i,1) = 1.7*v_a(1) - 0.7*v_i(i-1,1);
    v_i(i,2) = 1.7*v_a(2) - 0.7*v_i(i-1,2);
end
% Convert latitude and longitude to radius
L_k = rad2deg(L_k);
Lamda_k = rad2deg(Lamda_k);
% DR solution
DR = [t, L_k, Lamda_k, v_i, rad2deg(Psi_gyro_c)];

save('DR.mat','DR');

% Plotting the result
figure;
quiver(Lamda_k, L_k, v_i(:,2), v_i(:,1), 0.5);
xlabel('Longitude (deg)');
ylabel('Latitude (deg)');
title('DR Velocity Vector Field');
xlim([min(Lamda_k) max(Lamda_k)]);
ylim([min(L_k) max(L_k)]);
grid on;

% Plot the wheel speed
figure;
plot(t,Dead_reckoning(:, 2), 'g-','LineWidth', 1);
hold on;
plot(t,Dead_reckoning(:, 3), 'r-','LineWidth', 2);
hold on;
plot(t,Dead_reckoning(:, 4), 'b-','LineWidth', 1);
hold on;
plot(t,Dead_reckoning(:, 5), 'y-','LineWidth', 1);
hold on;
xlabel('time (s)');
ylabel('Speed of wheels (m/s)');
legend('Wheel-fl','Wheel-fr', 'Wheel-rl', 'Wheel-rr');
title('Speed of the wheels');
grid on;
hold off;
