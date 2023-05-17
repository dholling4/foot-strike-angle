clc
clear all
close all

trial_num = 3;

%%% Reading CSV files %%%
filepath2 = 'C:\Users\dholl\OneDrive\Documents\MECH 7710-Optimal_Control\Final Project\';
IMU = readmatrix(filepath2 + "running6mph_0" + string(trial_num) + ".csv");
angles = readmatrix(filepath2 + "run_0" + string(trial_num) +  ".csv");
ang_vel = readmatrix(filepath2 + "angvel_trial_0" + string(trial_num) + ".csv");

data_IMU = IMU;
data_MoC_pos = angles;
% [data_IMU,txt_IMU] = xlsread('IMU data file name.csv');
% [data_MoC_pos,txt_MoC_pos] = xlsread('Motion capture data file name.csv');


%%
clc

%%% Extracting data %%%
LA_Gyro = data_IMU(1:length(data_IMU),6);
LA_MoC_Pos  = data_MoC_pos(:,2);


%%% Downsampling the gyro and MoC data %%%
LA_Gyro = resample(LA_Gyro,1,18);

%%% Scaling and aligning data %%%
[peaks_g,locs_g] = findpeaks(-LA_Gyro*180,'MinPeakHeight',6);
[peaks_m,locs_m] = findpeaks(LA_MoC_Pos,'MinPeakHeight',6);
LA_Gyro = LA_Gyro(locs_g(1):length(LA_Gyro),:);
LA_MoC_Pos = LA_MoC_Pos(locs_m(1):length(LA_MoC_Pos),:);

LA_Gyro = [LA_Gyro;zeros(length(LA_MoC_Pos)-length(LA_Gyro),1)];
LA_MoC_Pos = [LA_MoC_Pos;zeros(length(LA_Gyro)-length(LA_MoC_Pos),1)];


%%% Synthesizing data %%%
data = [LA_MoC_Pos(1:3000),-LA_Gyro(1:3000)*pi/180];


%% The Kalman Filter
clc
clear Rd Qd A x1 x2 x3 P_ L P x_hat x_hat_
close all

%%% Kalman Filter %%%  % The states are [q dq B]
dt = 1/120;     % Sampling frequency

Rd = 0.0064;
Qd = [0.5 0 0;
      0 0.5 0;
      0 0 0.5];

A = [1 dt -dt;  % Integration Model
     0 1 -1;
     0 0 1];

x1(1) = 0;
x2(1) = 0;
x3(1) = 0;

H = [1 0 0;
     0 1 1];

x_hat_(:,1) = [x1;x2;x3];
P_(:,:,1) = A*[2,2,2;2,2,2;2,2,2]*A' + Qd;

for n = 1:length(data)

    % Update
    L(:,:,n) = P_(:,:,n)*H'*(H*P_(:,:,n)*H'+Rd)^-1;
    P(:,:,n) = (eye(3)-L(:,:,n)*H)*P_(:,:,n);
    x_hat(:,n) = x_hat_(:,n) + L(:,:,n)*([data(n,1);data(n,2)]-H*x_hat_(:,n));
    covar_m(n) = norm(P(:,:,n));

    % Break Loop to keep matrix dimensions consistent
    if n == length(data)
        break
    end

    % Propagate
    x1 = x_hat(1,n) + x_hat(2,n)*dt - x_hat(3,n)*dt;
    x2 = x_hat(2,n) - x_hat(3,n);
    x3 = x_hat(3,n);

    x_hat_(:,n+1) = [x1; x2; x3];
    P_(:,:,n+1) = A*P(:,:,n)*A' + Qd;
    
end

%% Plotting data and estimates
close all
clc

time_ = [0:0.02:60];

figure(1)
hold on
title('Angular position measurement vs. Estimated angular position trial 3')
ylabel('Degrees')
xlabel('time (s)')
grid on
plot(time_(1:500),x_hat(1,1:500),'+')
plot(time_(1:500),data(1:500,1))
legend('Position Estimate','Position Measurement')
savefig('C:\Users\dholl\OneDrive\Documents\MECH 7710-Optimal_Control\Final Project\figures\AngPos_measurementVsEstimated.fig')

figure(2)
hold on
title('Angular velocity measurement vs. Estimated angular velocity trial 3')
ylabel('Degrees/s')
xlabel('time (s)')
grid on
plot(time_(1:500),x_hat(2,1:500),'+')
plot(time_(1:500),data(1:500,2))
legend('Velocity Estimate','Velocity Measurement')
savefig('C:\Users\dholl\OneDrive\Documents\MECH 7710-Optimal_Control\Final Project\figures\AngVel_measurementVSEstimated.fig')

figure(3)
hold on
title('Bias estimate trial 3')
xlabel('time (s)')
ylabel('Degrees/s')
grid on
plot(time_(1:500),x_hat(3,1:500))
savefig('C:\Users\dholl\OneDrive\Documents\MECH 7710-Optimal_Control\Final Project\figures\Bias_Estimate.fig')


figure(4)
hold on
title('Norm of Covariance trial 3')
xlabel('time (s)')
grid on
plot(time_(1:500),covar_m(1:500))
savefig('C:\Users\dholl\OneDrive\Documents\MECH 7710-Optimal_Control\Final Project\figures\Covariance_Norm.fig')

xlswrite('P_values',covar_m(1:500),'A1:A500')
