% Gyro-Magnetometer Integration
clear
clc
format longg
Define_Constants;

%% Constants
tau_s = 0.5; % Time interval
sigma_gb = 1; % Gyroscope bias std deviation
sigma_gn = rad2deg(1e-4); % Gyroscope ramdom noise std deviation
sigma_mag = 4; % Magnetic compass noise error std deviation
S_rg = (3e-6)*rad_to_deg^2; % Gyroscope noise PSD
S_bgd = (2e-7)*rad_to_deg^2; % Gyroscope bias PSD (made up)

%% Extract data from tables
Dead_reckoning = readmatrix("Data\Dead_reckoning.csv");
t = Dead_reckoning(:,1); % length = 851
Omega = Dead_reckoning(:,6); % Angular rate in radius gyroscpoe
Omega = rad2deg(Omega); % Angular rate in degree
Psi_mag = Dead_reckoning(:,7); % Heading measurement from magnetic compass

%% Initialization
x = zeros(2,1); % Initial state
Psi_gyro = zeros(length(t),1); % Gyro-derived heading
Psi_gyro(1) = Psi_mag(1); % Use the initial heading from the compass
Psi_gyro_c = Psi_gyro; % Corrected gyroscope heading
% state estimation error covariance matrix
P = [sigma_gn^2,0;
     0,sigma_gb^2];
% State transition matrix
Phi = [1, tau_s;
       0, 1]; 
% System noise covariance
Q = [S_rg*tau_s+(S_bgd*tau_s^3)/3, 0.5*S_bgd*tau_s^2;
    0.5*S_bgd*tau_s^2, S_bgd*tau_s];
 % Measurement matrix
H = [-1, 0];
% Measurement noise covarince
R = sigma_mag^2;
epsilon_z_pre = 0;

%% Kalman filter iteration
for i = 2: length(t)
    Psi_gyro(i) = Psi_gyro(i-1) + Omega(i) * tau_s;
    % Propagate state estimate
    x_p = Phi*x;
    % Propagate error covariance
    P_p = Phi*P*Phi' + Q;
    % Kalman gain matrix
    K = P_p*H'/(H*P_p*H'+R);
    % Measurement innovation vector
    epsilon_z_pre = Psi_mag(i) - Psi_gyro(i) - H*x_p;
    % Update the state estimates
    x_u = x_p + K*epsilon_z_pre;
    % Update the error covariance matrix
    P_u = (1 - K*H)*P_p;
    % Correct the gyroscope solution with the Kalman filter
    Psi_gyro_c(i,:) = Psi_gyro(i,:) - x_u(1);
    % Update states and covariance for iteration
    x = x_u;
    P = P_u;
end

save('Integrated_Heading.mat','Psi_gyro_c');

% Plot the heading
figure;
plot(t,Psi_gyro, 'g--','LineWidth', 1);
hold on;
plot(t,Psi_mag, 'r--','LineWidth', 2);
hold on;
plot(t,Psi_gyro_c, 'b-','LineWidth', 1);
hold on;
xlabel('time (s)');
ylabel('Headings (degree)');
legend('Gyro-derived Heading','Magnetic Compass Measured Heading', 'Corrected Gyro Heading');
title('Gyro-Magnetometer Integration');
grid on;
hold off;