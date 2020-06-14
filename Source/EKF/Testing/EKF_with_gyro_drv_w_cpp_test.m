clc; clear all; close all;
%% Extended Kalman Filter (EKF) Driver Script

% This software is provided "as is" and any expressed or implied 
% warrantees, including, but not limited to, the implied warranties
% of merchantability and fitness for a particular purpose are disclaimed.
% Dr. Robert Bauer shall not be liable for any direct, indirect,
% consequential, or other damages suffered by anyone resulting from this
% work or the use of the research results/data of this work.


%% Load x_hat_kk results from C++ version
load x_hat_kk_cpp.txt
% x_hat_kk: state matrix values over many time instances (10x1)x100 matrix 
% array where 100 represents time instances and 10x1 consists of:
%                angular velocity (w)                     3x1
%                attitude quaternion (q vector portion)   3x1
%                attitude quaternion (q0 scalar portion)  1x1
%                gyro angular velocity bias (beta)        3x1


%% Load Test Data
% load the measurement variable "y" (7x251)
load y.txt
% each row of y corresponds to:
% 1. wx CubeSat angular velocity relative to ECI expressed in BF 
%       (x component) rad/s
% 2. wy CubeSat angular velocity relative to ECI expressed in BF 
%       (y component) rad/s
% 3. wz CubeSat angular velocity relative to ECI expressed in BF
%       (z component) rad/s
% 4. q1 element 1 of vector portion of quaternion representing rotation from ECI to BF
% 5. q2 element 2 of vector portion of quaternion representing rotation from ECI to BF
% 6. q3 element 3 of vector portion of quaternion representing rotation from ECI to BF
% 7. q0 scalar element quaternion representing rotation from ECI to BF

% load input variable "u" (3x251)
load u.txt
% each row of u corresponds to:
% 1. Tx applied torque x (in BF) Nm
% 2. Ty applied torque y (in BF) Nm
% 3. Tz applied torque z (in BF) Nm

% load "desired" variable (11x251) which contains the actual angular rates and
% quaternions and gyro angular rate bias which can be compared against 
% the estimates from the Kalman Filter
load desired.txt
% each column of desired corresponds to:
% 1. data time stamp, seconds (measurement sample rate is 1 second)
% 2. wx CubeSat angular velocity relative to ECI expressed in BF 
%       (x component) rad/s
% 3. wy CubeSat angular velocity relative to ECI expressed in BF 
%       (y component) rad/s
% 4. wz CubeSat angular velocity relative to ECI expressed in BF
%       (z component) rad/s
% 5. q1 element 1 of vector portion of quaternion representing rotation from ECI to BF
% 6. q2 element 2 of vector portion of quaternion representing rotation from ECI to BF
% 7. q3 element 3 of vector portion of quaternion representing rotation from ECI to BF
% 8. q0 scalar element quaternion representing rotation from ECI to BF
% 9. betax onboard gyro angular rate bias on BF x axis
%10. betay onboard gyro angular rate bias on BF y axis
%11. betaz onboard gyro angular rate bias on BF z axis

%% CubeSat dimensions for model in EKF
% assume 2U rectangular prism
dim_x=.1; % m
dim_y=.1; % m
dim_z=.2; % m

mass = 2; %kg
Ixx = 1/12*mass*(dim_y^2+dim_z^2); % kgm^2
Iyy = 1/12*mass*(dim_x^2+dim_z^2); % kgm^2
Izz = 1/12*mass*(dim_x^2+dim_y^2); % kgm^2

J=[Ixx 0   0;
    0  Iyy 0;
    0  0   Izz]; % exact inertia matrix of rectangular prism

%model_uncertainty = diag([1 1 1]); % no model uncertainty (100% accurate)
model_uncertainty = diag([1.1,1.1,0.90]); % 10 percent model uncertainty
J=J*model_uncertainty; % since in practice we do not have an exact model
                       % of the actual satellite we will assume some uncertainty

                       
tmax = 100;% s, simulation time
Ts = 1;    % s, sample time
n = 10;    % n states in state vector x
           % x=[wx; BF angular velocity relative to ECI expressed in BF
           %    wy;
           %    wz;
           %    q1; vector portion quaternion representing
           %        rotation from ECI to BF
           %    q2;
           %    q3;
           %    q0; scalar portion of quaternion representing 
           %        rotation from ECI to BF
           % betax; onboard gyro angular rate bias on BF x,y,z directions
           % betay; 
           % betaz];

%% Band-Limited White Gaussian Noise
% variance of noise signal
% standard deviation is sqrt(variance of noise signal)
% 68 percent of the generated noise is within plus/minus one standard deviation

% process noise added to equations of motion written in state space
% x_dot = .... + process noise
% variance of process noise on w_dot used to generate test data
phi_1_variance = 1E-6; %rad/s^2

% variance of process noise on q_dot used to generate test data
phi_2_variance = 1E-5; % rad/s

% variance of process noise on q0_dot used to generate test data
phi_3_variance = 1E-5; % rad/s

% variance of process noise on beta_dot used to generate test data
phi_4_variance = 1E-4; % rad/s

% assumed process noise covariance matrix Q which we give to Kalman Filter
% since in practice we don't know this exactly, we can try to estimate
% the values (here I multiply the actual values by 1.5)
Q = diag([phi_1_variance, phi_1_variance, phi_1_variance,...
          phi_2_variance, phi_2_variance, phi_2_variance,...
          phi_3_variance,...
          phi_4_variance, phi_4_variance, phi_4_variance]).*1.5;

% variance of measurement noise on w used to generate test data
psi_1_variance = 1E-4; %rad/s

% variance of measurement noise on q used to generate test data
psi_2_variance = 1E-3;

% variance of measurement noise on q0 used to generate test data
psi_3_variance = 1E-3;

% assumed measurement noise covariance matrix R which we give to Kalman Filter
% since in practice we don't know this exactly, we can try to estimate
% the values (here I multiply the actual values by 1.5)
R = diag([psi_1_variance, psi_1_variance, psi_1_variance,...
          psi_2_variance, psi_2_variance, psi_2_variance,...
          psi_3_variance]).*1.5;

%% Apply Extended Kalman Filter to the measurements
% assume initial conditions for state estimate (arbitrary values)
x_hat_kk(:,1) = [.04 -.02 -.03 .6 .2 -.1 .02 .05 -.1 -.15]';
% assume initial conditions for nxn covariance estimate
P_kk(:,:,1) = zeros(n,n);
% EKF iteration
for k = 2:tmax/Ts % loop through each measurement
 [x_hat_kk(:,k),P_kk(:,:,k)] = EKF_function_with_gyro(x_hat_kk(:,k-1),...
     P_kk(:,:,k-1),J,R,Q,Ts,u(:,k-1),y(:,k)); % EKF function
end

%% Plot Angular Velocity
figure(1)

subplot(3,1,1)
plot(desired(2,:),'k','LineWidth',.5)
hold on
plot(y(1,:),'b.')
stairs(x_hat_kk(1,:),'r-','LineWidth',1)
stairs(x_hat_kk_cpp(1,:),'g-','LineWidth',1)
ylabel('\omega_x (rad/s)','FontSize',12)
legend({'Actual','Biased Measurements','Kalman Filter', 'EKF C++'},'FontSize',10)
title('Angular Velocity','FontSize',14)
axis([0 tmax/Ts -.16 .16])

subplot(3,1,2)
plot(desired(3,:),'k','LineWidth',.5)
hold on
plot(y(2,:),'b.')
stairs(x_hat_kk(2,:),'r-','LineWidth',1)
stairs(x_hat_kk_cpp(2,:),'g-','LineWidth',1)
ylabel('\omega_y (rad/s)','FontSize',12)
axis([0 tmax/Ts -.16 .16])

subplot(3,1,3)
plot(desired(4,:),'k','LineWidth',.5)
hold on
plot(y(3,:),'b.')
stairs(x_hat_kk(3,:),'r-','LineWidth',1)
stairs(x_hat_kk_cpp(3,:),'g-','LineWidth',1)
ylabel('\omega_z (rad/s)','FontSize',12)
xlabel('Time (s)','FontSize',12)
axis([0 tmax/Ts -.16 .16])
set(gcf,'Position',[488,100,560,700]) % x,y,width,height

%% Plot Attitude Quaternion
figure(2)

subplot(4,1,1)
plot(desired(5,:),'k','LineWidth',.5)
hold on
plot(y(4,:),'b.')
stairs(x_hat_kk(4,:),'r-','LineWidth',1)
stairs(x_hat_kk_cpp(4,:),'g-','LineWidth',1)
ylabel('q_1','FontSize',12)
legend({'Actual','Measurements','Kalman Filter', 'EKF C++'},'FontSize',10)
title('Attitude Quaternion','FontSize',14)
axis([0 tmax/Ts -1 1])

subplot(4,1,2)
plot(desired(6,:),'k','LineWidth',.5)
hold on
plot(y(5,:),'b.')
stairs(x_hat_kk(5,:),'r-','LineWidth',1)
stairs(x_hat_kk_cpp(5,:),'g-','LineWidth',1)
ylabel('q_2','FontSize',12)
axis([0 tmax/Ts -1 1])

subplot(4,1,3)
plot(desired(7,:),'k','LineWidth',.5)
hold on
plot(y(6,:),'b.')
stairs(x_hat_kk(6,:),'r-','LineWidth',1)
stairs(x_hat_kk_cpp(6,:),'g-','LineWidth',1)
ylabel('q_3','FontSize',12)
axis([0 tmax/Ts -1 1])

subplot(4,1,4)
plot(desired(8,:),'k','LineWidth',.5)
hold on
plot(y(7,:),'b.')
stairs(x_hat_kk(7,:),'r-','LineWidth',1)
stairs(x_hat_kk_cpp(7,:),'g-','LineWidth',1)
ylabel('q_0','FontSize',12)
xlabel('Time (s)','FontSize',12)
axis([0 tmax/Ts -1 1])
set(gcf,'Position',[488,100,560,700]) % x,y,width,height

%% Plot Gyro Angular Rate Bias
figure(3)

subplot(3,1,1)
plot(desired(9,:),'k','LineWidth',.5)
hold on
stairs(x_hat_kk(8,:),'r-','LineWidth',1)
stairs(x_hat_kk_cpp(8,:),'g-','LineWidth',1)
ylabel('\beta_x (rad/s)','FontSize',12)
legend({'Actual','Kalman Filter', 'EKF C++'},'FontSize',10)
title('Gyro Angular Rate Bias','FontSize',14)
axis([0 tmax/Ts -.16 .16])


subplot(3,1,2)
plot(desired(10,:),'k','LineWidth',.5)
hold on
stairs(x_hat_kk(9,:),'r-','LineWidth',1)
stairs(x_hat_kk_cpp(9,:),'g-','LineWidth',1)
ylabel('\beta_y (rad/s)','FontSize',12)
axis([0 tmax/Ts -.16 .16])


subplot(3,1,3)
plot(desired(11,:),'k','LineWidth',.5)
hold on
stairs(x_hat_kk(10,:),'r-','LineWidth',1)
stairs(x_hat_kk_cpp(10,:),'g-','LineWidth',1)
ylabel('\beta_z (rad/s)','FontSize',12)
axis([0 tmax/Ts -.16 .16])
xlabel('Time (s)','FontSize',12)
set(gcf,'Position',[488,100,560,700]) % x,y,width,height
