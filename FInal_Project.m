%% MECH 7710 FINAL PROJECT
clear all; close all; clc
filepath='C:\Users\dholl\OneDrive\Documents\MECH 7710-Optimal_Control\Final Project\';
data = readmatrix(filepath + "rest_all_sensors_on_table01.csv");

gyro_X_LFT = data(4:end,6)* (pi/180);
gyro_X_RFT = data(4:end,15)* (pi/180);
gyro_X_LSK = data(4:end,24)* (pi/180);
gyro_X_RSK = data(4:end,33)* (pi/180);

mean_LFT = mean(gyro_X_LFT); stdev_LFT = std(gyro_X_LFT);
mean_RFT = mean(gyro_X_RFT); stdev_RFT = std(gyro_X_RFT);
mean_LSK = mean(gyro_X_LSK); stdev_LSK = std(gyro_X_LSK);
mean_RSK = mean(gyro_X_RSK); stdev_RSK = std(gyro_X_RSK);
figure; subplot(2,2,1); plot(gyro_X_LFT); title('LFoot Gyro \mu=' + string(round(mean_LFT,3)) + ' \sigma='+string(round(stdev_LFT,5)));
hold on; yline(mean_LFT, '--r', LineWidth=3); ylabel('Deg/s')
subplot(2,2,2); plot(gyro_X_RFT); title('RFoot Gyro \mu='+string(round(mean_RFT,3)) + ' \sigma=' +string(round(stdev_RFT,5)));
hold on; yline(mean_RFT, '--r', LineWidth=3);
subplot(2,2,3); plot(gyro_X_LSK); title('LShank Gyro \mu='+ string(round(mean_LSK,3)) + ' \sigma=' + string(round(stdev_LSK,5)));
hold on; yline(mean_LSK, '--r', LineWidth=3);
subplot(2,2,4); plot(gyro_X_RSK); title('RShank Gyro \mu=' + string(round(mean_RSK,3)) + ' \sigma=' + string(round(stdev_RSK,5)));
hold on; yline(mean_RSK, '--r', LineWidth=3);

gyro_cov = cov([gyro_X_LFT, gyro_X_RFT, gyro_X_LSK, gyro_X_RSK]); 
%% Get R: Measurement Noise Covariance Matrix for Running Trials
% close all; clear all; clc
trial_num = 3;
filepath2 = 'C:\Users\dholl\OneDrive\Documents\MECH 7710-Optimal_Control\Final Project\';
IMU = readmatrix(filepath2 + "running6mph_0" + string(trial_num) + ".csv");
angles = readmatrix(filepath2 + "run_0" + string(trial_num) +  ".csv");
ang_vel = readmatrix(filepath2 + "angvel_trial_0" + string(trial_num) + ".csv");
t = linspace(0, 60, size(IMU,1));

gyro_X_LFT = IMU(:,6);
gyro_X_RFT = IMU(:,15);
gyro_X_LSK = IMU(:,24);
gyro_X_RSK = IMU(:,33);

mean_LFT = mean(gyro_X_LFT); stdev_LFT = std(gyro_X_LFT);
mean_RFT = mean(gyro_X_RFT); stdev_RFT = std(gyro_X_RFT);
mean_LSK = mean(gyro_X_LSK); stdev_LSK = std(gyro_X_LSK);
mean_RSK = mean(gyro_X_RSK); stdev_RSK = std(gyro_X_RSK);

% Plot signals mean and stdev then get R
figure; subplot(2,2,1); plot(t, gyro_X_LFT); title('LFoot Gyro \mu=' + string(round(mean_LFT,3)) + ' \sigma='+string(round(stdev_LFT,2)));
hold on; yline(mean_LFT, '--r', LineWidth=3); ylabel('Deg/s')
subplot(2,2,2); plot(t, gyro_X_RFT); title('RFoot Gyro \mu='+string(round(mean_RFT,3)) + ' \sigma=' +string(round(stdev_RFT,2)));
hold on; yline(mean_RFT, '--r', LineWidth=3);
subplot(2,2,3); plot(t, gyro_X_LSK); title('LShank Gyro \mu='+ string(round(mean_LSK,3)) + ' \sigma=' + string(round(stdev_LSK,2)));
hold on; yline(mean_LSK, '--r', LineWidth=3); xlabel('Time (s)')
subplot(2,2,4); plot(t, gyro_X_RSK); title('RShank Gyro \mu=' + string(round(mean_RSK,3)) + ' \sigma=' + string(round(stdev_RSK,2)));
hold on; yline(mean_RSK, '--r', LineWidth=3); xlabel('Time (s)')

gyro_cov = cov([gyro_X_LFT, gyro_X_RFT, gyro_X_LSK, gyro_X_RSK]); 

%% % Load data from file (assuming it's stored in a CSV file)

% Extract necessary columns (assumingassuming data is organized as [time, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, foot_strike_angle])
foot_strike_angle = [angles(:, 2), angles(:, 2), angles(:, 2), angles(:, 2)];

% Define feature matrix X (using all measurements)
X = [gyro_X_LFT, gyro_X_RFT, gyro_X_LSK, gyro_X_RSK];
X = resample(X, size(foot_strike_angle,1), size(X,1));
%%
% close all; clc
fs = 60; % Hz
IMU = resample(IMU, size(angles,1), size(IMU,1));
acc_X = IMU(:, 3)/1000; acc_Y = IMU(:,4)/1000; acc_Z = IMU(:, 5)/1000;
acc_result = sqrt(acc_X.^2 + acc_Y.^2 + acc_Z.^2);
acc_result = resample(acc_result, size(angles,1), size(acc_result,1));
time = linspace(0,60, size(acc_result,1));

figure; subplot(2,1,1); plot(time, acc_X); hold on; plot(time, acc_Y); hold on; plot(time, acc_Z);
hold on; plot(time, acc_result); 
legend('AccX', 'AccY', 'AccZ', 'AccR');
subplot(2,1,2); plot(time, acc_result); title('Acc Resultant');
xlabel('Time (s)'); ylabel('m/s^2')

% Find peak acceleration to get time of foot strike
[max_acc, idx_acc] = findpeaks(acc_result, 'MinPeakProminence',20);
hold on; plot(time(idx_acc), max_acc,'ro');

%%
% close all; clc
gyro_X = IMU(:,6) * (pi/180); 
gyro_X = resample(gyro_X, size(angles,1), size(gyro_X,1));
length = size(gyro_X, 1)/2; % 30 sec slice

ang_vel_LSK = ang_vel(3:end, 2);
ang_vel_LFT = ang_vel(3:end, 8);
strike_vel = ang_vel_LFT - ang_vel_LSK;
% PLOT
time = linspace(0,60, size(ang_vel_LFT,1));
figure(1);
subplot(2,1,1); 
% plot(time, gyro_X(2:end,1)); 
title('Gyro');
hold on; plot(time, angles(2:end, 2));
hold on; plot(time(idx_acc), angles(idx_acc, 2), 'go');
legend('Mocap Angle', 'Foot Strike Angle')
xlabel('Time (s)')

figure(2)
subplot(3,1,1);
plot(time, ang_vel_LFT); title('Ang Vel Left Foot');
subplot(3,1,2);
plot(time, ang_vel_LSK); title('Ang Vel Left Shank');
subplot(3,1,3);
plot(time, strike_vel); title('Foot Strike Vel (Foot minus Shank)')
xlabel('Time (s)')

% INTEGRATE
strike_ang = cumsum(strike_vel) * (1/fs);
int_gyro = cumsum(gyro_X) * (1/fs);

[mins, idx] = findpeaks(-strike_ang,'MinPeakDistance', 60);
mins = -mins;
figure(3); title('Integrating Velocity to get Angles');
subplot(2,1,1); plot(time, strike_vel); 
title('Foot Strike Velocity'); ylabel('Deg/s'); 

subplot(2,1,2); 
plot(time, strike_ang); hold on; plot(time(idx), mins,'rx');
P = polyfit(idx, mins, 1);
yfit = polyval(P, idx);
hold on;
plot(time(idx), yfit,'r-.');
eqn = string(" Linear: y = " + 60*P(1)) + "x + " + string(P(2));
text(min(idx),max(mins),eqn,"HorizontalAlignment","center","VerticalAlignment","bottom");
title('Integrated Foot Strike Angle')
xlabel('Time (s)')

figure(1); subplot(2,1,2); plot(time, int_gyro(2:end));
title('Integrated Gyro of the Foot');
ylabel('Foot Angle (Deg)'); xlabel('time (s)');
[min_gyro, idx_gyro] = findpeaks(int_gyro, 'MinPeakProminence',2);
hold on; plot(time(idx_gyro), min_gyro, 'rx');
hold on; plot(time(idx_acc), int_gyro(idx_acc), 'go')

P_gyro = polyfit(idx_gyro, min_gyro, 1);
yfit_gyro = polyval(P_gyro, idx_gyro);
hold on; 
plot(time(idx_gyro), yfit_gyro, 'r-.');
eqn = string(" Linear: y = " + 60*P_gyro(1)) + "x + " + string(P_gyro(2));
text(min(idx_gyro),max(min_gyro),eqn,"HorizontalAlignment","center","VerticalAlignment","bottom");

% [min_angle, idx_angle] = findpeaks(angles)
bias_gyro = 60*P_gyro(1);

%% METRICS
% Compare Ground Truth Foot strike angle from mocap to estimations from Gyro
fontsize=14;
y = angles(idx_acc, 2);
y_hat = int_gyro(idx_acc);
error = y_hat - y;
figure; 
subplot(2,2,1);
hist(error, 15); xlabel('estimation error (deg)', 'FontSize', fontsize)
title('Footstrike Estimation Error');
ax = gca;
ax.FontSize = 26;

subplot(2,2,2);
bar(error); xlabel('Stride Number');
ylabel('Estimation Error (deg)', 'FontSize',fontsize)
title('Footstrike Estimation Error: MAE='+ string(mean(abs(error))) + ' stdev=' + string(std(error)));
ax = gca;
ax.FontSize = 26;

% Remove Gyro Drift
y_hat_corrected = zeros(size(y_hat,1),1);
for N=1:size(y_hat,1)
    y_hat_corrected(N) = int_gyro(idx_acc(N)) - N*bias_gyro;
end

error_corrected = y_hat_corrected - y;

subplot(2,2,3);
hist(error_corrected, 15); xlabel('estimation error (deg)','FontSize',fontsize)
title('Gyro Drift Corrected');
ax = gca;
ax.FontSize = 26;

subplot(2,2,4); bar(error_corrected); xlabel('Stride Number','FontSize',fontsize);
ylabel('Estimation Error (deg)', 'FontSize', fontsize)
title('Gyro Drift Corrected: MAE='+ string(mean(abs(error_corrected))) + ' stdev=' + string(std(error_corrected)));
ax = gca;
ax.FontSize = fontsize;
