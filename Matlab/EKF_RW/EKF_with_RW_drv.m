clc; clear all; close all;
%% Extended Kalman Filter (EKF) Driver Script WITH REACTION WHEELS (RWs)

% This software is provided "as is" and any expressed or implied 
% warrantees, including, but not limited to, the implied warranties
% of merchantability and fitness for a particular purpose are disclaimed.
% Dr. Robert Bauer shall not be liable for any direct, indirect,
% consequential, or other damages suffered by anyone resulting from this
% work or the use of the research results/data of this work.

%% Load Test Data (measurement sample rate is 1Hz)
% load the measurement variable "y" (10x251)
load y.txt
y=y';
% each row of y corresponds to:
% 1. OMEGAx RW angular velocity of x RW relative to BF expressed in
%    body-fixed (BF) axes
% 2. OMEGAy RW angular velocity of y RW relative to BF expressed in BF
% 3. OMEGAz RW angular velocity of z RW relative to BF expressed in BF
% 4. wx CubeSat angular velocity relative to Earth-Centered-Inertia (ECI) expressed in BF 
%       (x component) rad/s
% 5. wy CubeSat angular velocity relative to ECI expressed in BF 
%       (y component) rad/s
% 6. wz CubeSat angular velocity relative to ECI expressed in BF
%       (z component) rad/s
% 7. q1 element 1 of vector portion of quaternion representing rotation from ECI to BF
% 8. q2 element 2 of vector portion of quaternion representing rotation from ECI to BF
% 9. q3 element 3 of vector portion of quaternion representing rotation from ECI to BF
%10. q0 scalar element quaternion representing rotation from ECI to BF

% load input variable "u" (6x251)
load u.txt
u=u';
% each row of u corresponds to:
% 1. Tx applied external torque x (in BF) Nm
% 2. Ty applied external torque y (in BF) Nm
% 3. Tz applied external torque z (in BF) Nm
% 4. RWx applied RW torque x (in BF) Nm
% 5. RWy applied RW torque y (in BF) Nm
% 6. RWz applied RW torque z (in BF) Nm

% load "actual" and "KF_desired" variables, each of dimension (13x251) 
% "actual.txt" contains the actual RW and body angular rates, quaternions
% and gyro angular rate bias which can be compared against 
% the estimates from the Kalman Filter
% "KF_desired.txt" contains the Kalman Filter estimates from Simulink
% which I used to check this code, so I commented it out
load actual.txt
actual=actual';
% load KF_desired.txt %I just used this data to check
% KF_desired=KF_desired';

% each column of desired corresponds to:
% 1. OMEGAx RW angular velocity of x RW relative to BF expressed in BF
% 2. OMEGAy RW angular velocity of y RW relative to BF expressed in BF
% 3. OMEGAz RW angular velocity of z RW relative to BF expressed in BF
% 4. wx CubeSat angular velocity relative to ECI expressed in BF 
%       (x component) rad/s
% 5. wy CubeSat angular velocity relative to ECI expressed in BF 
%       (y component) rad/s
% 6. wz CubeSat angular velocity relative to ECI expressed in BF
%       (z component) rad/s
% 7. q1 element 1 of vector portion of quaternion representing rotation from ECI to BF
% 8. q2 element 2 of vector portion of quaternion representing rotation from ECI to BF
% 9. q3 element 3 of vector portion of quaternion representing rotation from ECI to BF
%10. q0 scalar element quaternion representing rotation from ECI to BF
%11. betax onboard gyro angular rate bias on BF x axis
%12. betay onboard gyro angular rate bias on BF y axis
%13. betaz onboard gyro angular rate bias on BF z axis

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
%assume Reaction Wheels are cylinders concentrated at the COM of the
%satellite
r=0.02; %m
h=0.01; %m
rho = 8730; %kg/m^3
mw = pi*r^2*h*rho; %kg
Jwx = 1/2*mw*r^2; %kgm^2, axial moment of inertia of RWs
Jwy = Jwx;
Jwz = Jwx;
Jw = [Jwx 0    0;
      0    Jwy 0;
      0    0    Jwz]; % RW moment of inertia matrix **assuming that all 
                      % are located at the COM of rectangular prism and aligned with BF
                      % (this is a modeling approximation)

%Add RW moments of inertia about non-spin axes to satellite inertia:
Iwt=mw*(3*(2*r)^2+4*h^2)/48; %transverse moment of inertia of RWs
J(1,1) = J(1,1) + 2*Iwt; %add effect if Iwt for y and z RWs
J(2,2) = J(2,2) + 2*Iwt; %add effect of Iwt for x and z RWs
J(3,3) = J(3,3) + 2*Iwt; %add effect if Iwt for x and y RWs

%model_uncertainty = diag([1 1 1]); % no model uncertainty (100% accurate)
model_uncertainty = diag([1.1,1.1,0.90]); % 10 percent model uncertainty
J=J*model_uncertainty; % since in practice we do not have an exact model
                       % of the actual satellite we will assume some uncertainty
                       
tmax = 250;% s, simulation time
Ts = 1;    % s, sample time
n = 13;    % n states in state vector x
           % x=[OMEGAx; RW angular velocity relative to BF expressed in BF
           %    OMEGAy; [rad/s]
           %    OMEGAz;
           %        wx; BF angular velocity relative to ECI expressed in BF
           %        wy; [rad/s]
           %        wz;
           %        q1; vector portion quaternion representing
           %            rotation from ECI to BF
           %        q2;
           %        q3;
           %        q0; scalar portion of quaternion representing 
           %            rotation from ECI to BF
           %     betax; onboard gyro angular rate bias on BF x,y,z directions
           %     betay; [rad/s]
           %     betaz];

%% Band-Limited White Gaussian Noise
% variance of noise signal
% standard deviation is sqrt(variance of noise signal)
% 68 percent of the generated noise is within plus/minus one standard deviation

% process noise added to equations of motion written in state space
% x_dot = .... + process noise
% variance of process noise on OMEGA_dot used to generate test data
phi_0_variance = 1E-1; %rad/s^2

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
Q = diag([phi_0_variance, phi_0_variance, phi_0_variance,...
          phi_1_variance, phi_1_variance, phi_1_variance,...
          phi_2_variance, phi_2_variance, phi_2_variance,...
          phi_3_variance,...
          phi_4_variance, phi_4_variance, phi_4_variance]);%.*1.5;

% variance of measurement noise on OMEGA used to generate test data
psi_0_variance = 1E-1; %rad/s

% variance of measurement noise on w used to generate test data
psi_1_variance = 1E-4; %rad/s

% variance of measurement noise on q used to generate test data
psi_2_variance = 1E-3;

% variance of measurement noise on q0 used to generate test data
psi_3_variance = 1E-3;

% assumed measurement noise covariance matrix R which we give to Kalman Filter
% since in practice we don't know this exactly, we can try to estimate
% the values (here I multiply the actual values by 1.5)
R = diag([psi_0_variance, psi_0_variance, psi_0_variance,...
          psi_1_variance, psi_1_variance, psi_1_variance,...
          psi_2_variance, psi_2_variance, psi_2_variance,...
          psi_3_variance]);%.*1.5;

%% Apply Extended Kalman Filter to the measurements
% assume initial conditions for state estimate (arbitrary values)
x_hat_kk(:,1) = [12 -10 8 .04 -.02 -.03 .6 .2 -.1 .02 .05 -.1 -.15]';
% assume initial conditions for nxn covariance estimate
P_kk(:,:,1) = zeros(n,n);
% EKF iteration

for k = 2:tmax/Ts % loop through each measurement
    
    [x_hat_kk(:,k),P_kk(:,:,k)] = EKF_function_with_RW(x_hat_kk(:,k-1),...
     P_kk(:,:,k-1),J,Jw,R,Q,Ts,u(1:3,k-1),u(4:6,k-1),y(:,k)); % EKF function
end

%% Plot RW Angular Velocity
figure(1)
subplot(3,1,1)
plot(actual(1,:),'k','LineWidth',.5)
hold on
plot(y(1,:),'b.')
stairs(x_hat_kk(1,:),'r-','LineWidth',1)
%stairs(KF_desired(1,:),'m--','LineWidth',.5)
ylabel('\Omega_x (rad/s)','FontSize',12)
legend({'Actual','Measurements','Kalman Filter'},'FontSize',10)
title('RW Angular Velocity','FontSize',14)
axis([0 tmax/Ts -5 20])
subplot(3,1,2)
plot(actual(2,:),'k','LineWidth',.5)
hold on
plot(y(2,:),'b.')
stairs(x_hat_kk(2,:),'r-','LineWidth',1)
%stairs(KF_desired(2,:),'m--','LineWidth',.5)
ylabel('\Omega_y (rad/s)','FontSize',12)
axis([0 tmax/Ts -5 20])
subplot(3,1,3)
plot(actual(3,:),'k','LineWidth',.5)
hold on
plot(y(3,:),'b.')
stairs(x_hat_kk(3,:),'r-','LineWidth',1)
%stairs(KF_desired(3,:),'m--','LineWidth',.5)
ylabel('\Omega_z (rad/s)','FontSize',12)
xlabel('Time (s)','FontSize',12)
axis([0 tmax/Ts -5 20])
set(gcf,'Position',[488,100,560,700]) % x,y,width,height


%% Plot Satellite Angular Velocity
figure(2)
subplot(3,1,1)
plot(actual(4,:),'k','LineWidth',.5)
hold on
plot(y(4,:),'b.')
stairs(x_hat_kk(4,:),'r-','LineWidth',1)
%stairs(KF_desired(4,:),'m--','LineWidth',.5)
ylabel('\omega_x (rad/s)','FontSize',12)
legend({'Actual','Biased Measurements','Kalman Filter'},'FontSize',10)
title('Satellite Angular Velocity','FontSize',14)
axis([0 tmax/Ts -.16 .16])
subplot(3,1,2)
plot(actual(5,:),'k','LineWidth',.5)
hold on
plot(y(5,:),'b.')
stairs(x_hat_kk(5,:),'r-','LineWidth',1)
%stairs(KF_desired(5,:),'m--','LineWidth',.5)
ylabel('\omega_y (rad/s)','FontSize',12)
axis([0 tmax/Ts -.16 .16])
subplot(3,1,3)
plot(actual(6,:),'k','LineWidth',.5)
hold on
plot(y(6,:),'b.')
stairs(x_hat_kk(6,:),'r-','LineWidth',1)
%stairs(KF_desired(6,:),'m--','LineWidth',.5)
ylabel('\omega_z (rad/s)','FontSize',12)
xlabel('Time (s)','FontSize',12)
axis([0 tmax/Ts -.16 .16])
set(gcf,'Position',[488,100,560,700]) % x,y,width,height

%% Plot Attitude Quaternion
figure(3)
subplot(4,1,1)
plot(actual(7,:),'k','LineWidth',.5)
hold on
plot(y(7,:),'b.')
stairs(x_hat_kk(7,:),'r-','LineWidth',1)
%stairs(KF_desired(7,:),'m--','LineWidth',.5)
ylabel('q_1','FontSize',12)
legend({'Actual','Measurements','Kalman Filter'},'FontSize',10)
title('Attitude Quaternion','FontSize',14)
axis([0 tmax/Ts -1 1])
subplot(4,1,2)
plot(actual(8,:),'k','LineWidth',.5)
hold on
plot(y(8,:),'b.')
stairs(x_hat_kk(8,:),'r-','LineWidth',1)
%stairs(KF_desired(8,:),'m--','LineWidth',.5)
ylabel('q_2','FontSize',12)
axis([0 tmax/Ts -1 1])
subplot(4,1,3)
plot(actual(9,:),'k','LineWidth',.5)
hold on
plot(y(9,:),'b.')
stairs(x_hat_kk(9,:),'r-','LineWidth',1)
%stairs(KF_desired(9,:),'m--','LineWidth',.5)
ylabel('q_3','FontSize',12)
axis([0 tmax/Ts -1 1])
subplot(4,1,4)
plot(actual(10,:),'k','LineWidth',.5)
hold on
plot(y(10,:),'b.')
stairs(x_hat_kk(10,:),'r-','LineWidth',1)
%stairs(KF_desired(10,:),'m--','LineWidth',.5)
ylabel('q_0','FontSize',12)
xlabel('Time (s)','FontSize',12)
axis([0 tmax/Ts -1 1])
set(gcf,'Position',[488,100,560,700]) % x,y,width,height

%% Plot Gyro Angular Rate Bias
figure(4)
subplot(3,1,1)
plot(actual(11,:),'k','LineWidth',.5)
hold on
stairs(x_hat_kk(11,:),'r-','LineWidth',1)
%stairs(KF_desired(11,:),'m--','LineWidth',.5)
ylabel('\beta_x (rad/s)','FontSize',12)
legend({'Actual','Kalman Filter'},'FontSize',10)
title('Gyro Angular Rate Bias','FontSize',14)
axis([0 tmax/Ts -.25 .16])
subplot(3,1,2)
plot(actual(12,:),'k','LineWidth',.5)
hold on
stairs(x_hat_kk(12,:),'r-','LineWidth',1)
%stairs(KF_desired(12,:),'m--','LineWidth',.5)
ylabel('\beta_y (rad/s)','FontSize',12)
axis([0 tmax/Ts -.25 .16])
subplot(3,1,3)
plot(actual(13,:),'k','LineWidth',.5)
hold on
stairs(x_hat_kk(13,:),'r-','LineWidth',1)
%stairs(KF_desired(13,:),'m--','LineWidth',.5)
ylabel('\beta_z (rad/s)','FontSize',12)
axis([0 tmax/Ts -.25 .16])
xlabel('Time (s)','FontSize',12)
set(gcf,'Position',[488,100,560,700]) % x,y,width,height

% Print results
dlmwrite('x_hat_kk_matlab.txt', x_hat_kk, 'delimiter', ' ', 'precision', 15);
dlmwrite('P_kk_matlab.txt', P_kk, 'delimiter', ' ', 'precision', 15);

