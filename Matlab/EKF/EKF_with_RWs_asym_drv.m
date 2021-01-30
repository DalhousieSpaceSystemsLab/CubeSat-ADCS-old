clc; clear all; close all;
%% Extended Kalman Filter (EKF) Driver Script WITH ASYMMETRIC REACTION WHEELS (RWs)
% The CubeSat inertia matrix does not need to be diagonal
% The RWs do not need to be located at the center of mass (COM)
%
% This software is provided "as is" and any expressed or implied 
% warrantees, including, but not limited to, the implied warranties
% of merchantability and fitness for a particular purpose are disclaimed.
% Dr. Robert Bauer shall not be liable for any direct, indirect,
% consequential, or other damages suffered by anyone resulting from this
% work or the use of the research results/data of this work.
%
% BF = body-fixed frame, fixed to satellite at COM of CubeSat
%ECI = Earth-Centered-Inertial frame
%% Load Simulation Test Data (measurement sample rate is 1Hz)
% load the measurement variable "y" (10x501)
load y.txt
y=y';
% each row of y corresponds to:
% 1. ws1 scalar angular speed of RW#1 relative to BF
% 2. ws2 scalar angular speed of RW#2 relative to BF
% 3. ws3 scalar angular speed of RW#3 relative to BF
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

% load input variable "u" (6x501)
load u.txt
u=u';
% each row of u corresponds to:
% 1. gx applied external torque x (in BF) Nm
% 2. gy applied external torque y (in BF) Nm
% 3. gz applied external torque z (in BF) Nm
% 4. ga1 applied scalar torque from motor on RW#1 Nm
% 5. ga2 applied scalar torque from motor on RW#2 Nm
% 6. ga3 applied scalar torque from motor on RW#3 Nm

% load "actual" and "KF_desired" variables, each of dimension (13x501) 
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
% 1. ws1 scalar angular speed of RW#1 relative to BF
% 2. ws2 scalar angular speed of RW#2 relative to BF
% 3. ws3 scalar angular speed of RW#3 relative to BF
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
%  2U CubeSat with asymmetric solar arrays and RWs
%  Fa is Earth-Centered-Inertial (ECI) frame
%  Fb is body-fixed (BF) frame fixed to CubeSat at point O which
%  corresponds to the center of mass (COM) of entire system
%  (ie. combined COM of R+W1+W2+W3)
%  CubeSat consists of rigid body R (without rotors) PLUS
%  three RWs with rotors Wi spinning about axis ai, i=1,2,3
%            W1 along a1=[1,0,0] in BF, vector from O to COM of W1 is b1
%            W2 along a2=[0,1,0] in BF, vector from O to COM of W2 is b2
%            W3 along a3=[0,0,1] in BF, vector from O to COM of W3 is b3
%      __________________
%     /                 /|  4 solar arrays not shown here
%    /                 / |
%   /________b________/  |
%   |        ___  __W1|  |
%   |     W3|___|/  \ |  |
%   | W2         \ _/ |  |
%   | /\     z|       |  |
%   ||  |     | /x    |  |c
%   ||  | ____|/      |  |
%   | \/  y   O Fb    |  |
%   |                 |  |
%   |                 |  |
%   |                 |  |
%   |                 |  |
%   | rigid body R    |  |
%   |                 |  /
%   |     /-> A       | /a 
%   |____/____________|/ 
%       /   
%      Frame A (reference frame with origin at center of bottom plane of CubeSat
%               used to determine the location of O), axes aligned with Fb
%
%      z
%      |  x
%      | / 
% y____|/ Fa
%
% dimensions of 2U rectangular prism R [m] (a=length, b=height, c=depth)
a=.1;   %m (along BF x axis)
b=.1;   %m (along BF y axis)
c=.2;   %m (along BF z axis)

% dimension of cylindrical-shaped RW rotor Wi
% THESE DIMENSIONS WILL NOT BE NEEDED IN THE FINAL DESIGN SINCE THE ROTOR
% WILL LIKELY NOT BE A SIMPLE CYLINDER
r_rotor = .02; %m, radius
l_rotor = .01; %m, width

% define rotor spin axes ai expressed in BF
a1_b = [1,0,0]';
a2_b = [0,1,0]';
a3_b = [0,0,1]';

% mass of CubeSat rigid body R (without RW rotors)
% VALUE FOR m_R WILL NEED TO BE ADJUSTED TO MATCH FINAL DESIGN
m_R = 3.7999999999999976;  % kg, value from SolidWorks

% mass of cylindrical-shaped RW rotor Wi
% VALUE FOR m_W WILL NEED TO BE ADJUSTED TO MATCH FINAL DESIGN
rho_W = 1000;                      %kg/m^3, density of rotor Wi
m_W=pi*r_rotor^2*l_rotor*rho_W;    %kg, mass of rotor Wi

% moments of inertia MoI and products of inertia (PoI)
% of CubeSat R (without RW rotors) taken wrt COM of R (without RW rotors) 
% about axes parallel to Fb 
% NOTE: PoI needed is the NEGATIVE of what is calculated by
% SolidWorks since SolidWorks uses their own sign convention as follows:
%
% SolidWorks (SW) calculates:
%          /                 /                 /
% Ixy_SW = |(xy)dm, Iyz_SW = |(yz)dm, Izx_SW = |(zx)dm
%          /                 /                 /
% and the inertia tensor matrix is:
%
% [  Ixx_SW -Ixy_SW  -Ixz_SW ]      [ Ixx  Ixy  Ixz ]
% [ -Ixy_SW  Iyy_SW  -Iyz_SW ]   =  [ Ixy  Iyy  Iyz ]
% [ -Ixz_SW -Iyz_SW   Izz_SW ]      [ Ixz  Iyz  Izz ]
%                                           /\
%                                           ||
%                                    where the RHS are the traditional
%                                    inertia tensor components
%
% So, while SolidWorks (for this example) calculates the inertia tensor 
% elements taken at the center of mass (without the rotors) and aligned 
% with the output coordinate system (which is aligned with BF) as:
% [ 0.035945701754385918  -0.0014210526315789342 -0.0031263157894736802;
%  -0.0014210526315789342  0.039761491228070128  -0.0023842105263157877;
%  -0.0031263157894736802 -0.0023842105263157877  0.04825438596491223];
%
% we, instead, use the following for the inertia tensor of R (without rotors)
% taken wrt COM about axes parallel to Fb:
% VALUE FOR J_R WILL NEED TO BE ADJUSTED TO MATCH FINAL DESIGN
J_R = [0.035945701754385918  0.0014210526315789342 0.0031263157894736802;
       0.0014210526315789342 0.039761491228070128  0.0023842105263157877;
       0.0031263157894736802 0.0023842105263157877 0.04825438596491223];

% location of COM of CubeSat R (without rotors) from point A expressed in Fb
% VALUES FOR rAR_b, rAW1_b, rAW2_b and rAW3_b WILL NEED TO BE ADJUSTED TO 
% MATCH FINAL DESIGN
rAR_b = [-0.023684210526315773;
         -0.015789473684210509;
          0.14026315789473687];  % m

% location of COM of W1 from point A expressed in Fb
rAW1_b = [3/4*a/2;
          b/4;
          3/4*c]; 

% location of COM of W2 from point A expressed in Fb 
rAW2_b = [0;
          3/4*b/2;
          3/4*c];  

% location of COM of W3 from point A expressed in Fb    
rAW3_b = [0;
          0;
          7/8*c]; 
      
% location of O (COM of combined R1&R2&W1&W2&W3 from point A) expressed in Fb
rAO_b = (1/(m_R+3*m_W))*(m_R*rAR_b + m_W*rAW1_b + m_W*rAW2_b + m_W*rAW3_b );

r_b =  rAR_b  - rAO_b;  % location of COM of R (without rotors) from O
b1_b = rAW1_b - rAO_b;  % location of COM of W1 from O
b2_b = rAW2_b - rAO_b;  % location of COM of W2 from O
b3_b = rAW3_b - rAO_b;  % location of COM of W3 from O

% use Parallel Axis Theorem to calculate moment of inertia of R wrt O
% wrt Fb axes, kgm^2
J_b = J_R+m_R*(norm(r_b,2)^2*eye(3,3)-r_b*r_b');

% moment of inertia of W wrt W's COM about its axis of symmetry, kgm^2
% VALUE for Is WILL NEED TO BE ADJUSTED TO MATCH FINAL DESIGN
Is=1/2*m_W*r_rotor^2; 

% moment of inertia of W wrt W's COM about any transverse axis, kgm^2
% VALUE for It WILL NEED TO BE ADJUSTED TO MATCH FINAL DESIGN
It=1/12*m_W*(3*r_rotor^2+l_rotor^2);

% moment of inertia of W1 wrt W's COM wrt Fb axes
IW1_b = It*eye(3,3)+(Is-It)*(a1_b*a1_b');  
% moment of inertia of W1 wrt W's COM wrt Fb axes
IW2_b = It*eye(3,3)+(Is-It)*(a2_b*a2_b');  
% moment of inertia of W1 wrt W's COM wrt Fb axes
IW3_b = It*eye(3,3)+(Is-It)*(a3_b*a3_b');  

% define second moment of inertia of entire system (R plus three rotors)
% about O (COM of entire system)
J=J_b+IW1_b+IW2_b+IW3_b+...
    m_W*(norm(b1_b,2)^2*eye(3,3)-b1_b*b1_b')+...
    m_W*(norm(b2_b,2)^2*eye(3,3)-b2_b*b2_b')+...
    m_W*(norm(b3_b,2)^2*eye(3,3)-b3_b*b3_b');

%can add model uncertainty for KF
model_uncertainty = 1.05; % 5% uncertainty
%model_uncertainty = 1.0; % no uncertainty
J_KF=J*model_uncertainty;

%moment of inertia that needs to be inverted for angular
%acceleration calculation in KF
Jnew_KF=J_KF-Is*(a1_b*a1_b')-Is*(a2_b*a2_b')-Is*(a3_b*a3_b');
Jnew_inv_KF = inv(Jnew_KF);

tmax = 500;%s, simulation time
Ts = 1;    %s, sample time
n=13;      % n states in state vector x
           % x=[ha1; component along rotor spin axis of absolute angular
           %    ha2; momentum of RWi about COM of Wi [Nms]
           %    ha3;
           %     wx; BF angular velocity relative to ECI expressed in BF
           %     wy; [rad/s]
           %     wz;
           %     q1; vector portion quaternion representing
           %         rotation from ECI to BF
           %     q2;
           %     q3;
           %     q0; scalar portion of quaternion representing 
           %         rotation from ECI to BF
           %  betax; onboard gyro angular rate bias on BF x,y,z directions
           %  betay; [rad/s]
           %  betaz];
%% Band-Limited White Gaussian Noise
% variance of noise signal
% standard deviation is sqrt(variance of noise signal)
% 68 percent of the generated noise is within plus/minus one standard deviation

% process noise added to equations of motion written in state space
% x_dot = .... + process noise

% variance of process noise on hai_dot used to generate test data
phi_0_variance = 1E-5; %Nm

% variance of process noise on w_dot used to generate test data
phi_1_variance = 1E-8; %rad/s^2

% variance of process noise on q_dot used to generate test data
phi_2_variance = 1E-5; %rad/s

% variance of process noise on q0_dot used to generate test data
phi_3_variance = 1E-5; %rad/s

% variance of process noise on beta_dot used to generate test data
phi_4_variance = 1E-4; %rad/s

% assumed process noise covariance matrix Q which we give to Kalman Filter
% since in practice we don't know this exactly, we can try to estimate
% the values (here I provide the option to multiply the actual values by 1.5)
Q = diag([phi_0_variance, phi_0_variance, phi_0_variance,...
          phi_1_variance, phi_1_variance, phi_1_variance,...
          phi_2_variance, phi_2_variance, phi_2_variance,...
          phi_3_variance,...
          phi_4_variance, phi_4_variance, phi_4_variance]); %.*1.5;

% variance of measurement noise on RW speeds wsi used to generate test data
psi_0_variance = 1E-2; %rad/s

% variance of measurement noise on w used to generate test data
psi_1_variance = 1E-5; %rad/s

% variance of measurement noise on q used to generate test data
psi_2_variance = 1E-2;

% variance of measurement noise on q0 used to generate test data
psi_3_variance = 1E-2;

% assumed measurement noise covariance matrix R which we give to Kalman Filter
% since in practice we don't know this exactly, we can try to estimate
% the values (here I provide the option to multiply the actual values by 1.5)

R = diag([psi_0_variance, psi_0_variance, psi_0_variance,...
          psi_1_variance, psi_1_variance, psi_1_variance,...
          psi_2_variance, psi_2_variance, psi_2_variance,...
          psi_3_variance]); %.*1.5;

%% Apply Extended Kalman Filter to the measurements
% assume initial conditions for state estimate (arbitrary values)
x_hat_kk(:,1) = [-.02 .04 .01 -.01 .01 .002 .5 -.25 .3 .8 .1 -.1 .15]';
% assume initial conditions for nxn covariance estimate
P_kk(:,:,1) = zeros(n,n);
wsi_kk(:,1) = zeros(3,1);
% EKF iteration
for k = 2:tmax/Ts % loop through each measurement
 [x_hat_kk(:,k),P_kk(:,:,k),wsi_kk(:,k)] = EKF_function_with_RWs_asym(J_KF,Jnew_inv_KF,Is,...
    a1_b,a2_b,a3_b,R,Q,Ts,u(1:3,k-1),u(4:6,k-1),y(:,k),x_hat_kk(:,k-1),P_kk(:,:,k-1)); % EKF function
end

%% Plot RW Angular Velocity
figure(1)
subplot(3,1,1)
plot(actual(1,:),'k','LineWidth',.5)
hold on
plot(y(1,:),'b.')
stairs(wsi_kk(1,:),'r-','LineWidth',1)
% stairs(KF_desired(1,:),'g--','LineWidth',.5)
ylabel('\omega_s_1 (rad/s)','FontSize',12)
legend({'Actual','Measurements','Kalman Filter'},'FontSize',10)
title('RW Angular Velocity','FontSize',14)
axis([0 tmax/Ts -5 1000])
subplot(3,1,2)
plot(actual(2,:),'k','LineWidth',.5)
hold on
plot(y(2,:),'b.')
stairs(wsi_kk(2,:),'r-','LineWidth',1)
% stairs(KF_desired(2,:),'g--','LineWidth',.5)
ylabel('\omega_s_2 (rad/s)','FontSize',12)
axis([0 tmax/Ts -1000 5])
subplot(3,1,3)
plot(actual(3,:),'k','LineWidth',.5)
hold on
plot(y(3,:),'b.')
stairs(wsi_kk(3,:),'r-','LineWidth',1)
% stairs(KF_desired(3,:),'g--','LineWidth',.5)
ylabel('\omega_s_3 (rad/s)','FontSize',12)
xlabel('Time (s)','FontSize',12)
axis([0 tmax/Ts -5 1000])
set(gcf,'Position',[488,100,560,700]) % x,y,width,height


%% Plot Satellite Angular Velocity
figure(2)
subplot(3,1,1)
plot(actual(4,:),'k','LineWidth',.5)
hold on
plot(y(4,:),'b.')
stairs(x_hat_kk(4,:),'r-','LineWidth',1)
% stairs(KF_desired(4,:),'g--','LineWidth',.5)
ylabel('\omega_x (rad/s)','FontSize',12)
legend({'Actual','Biased Measurements','Kalman Filter'},'FontSize',10)
title('Satellite Angular Velocity','FontSize',14)
axis([0 tmax/Ts -.16 .16])
subplot(3,1,2)
plot(actual(5,:),'k','LineWidth',.5)
hold on
plot(y(5,:),'b.')
stairs(x_hat_kk(5,:),'r-','LineWidth',1)
% stairs(KF_desired(5,:),'g--','LineWidth',.5)
ylabel('\omega_y (rad/s)','FontSize',12)
axis([0 tmax/Ts -.16 .16])
subplot(3,1,3)
plot(actual(6,:),'k','LineWidth',.5)
hold on
plot(y(6,:),'b.')
stairs(x_hat_kk(6,:),'r-','LineWidth',1)
% stairs(KF_desired(6,:),'g--','LineWidth',.5)
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
% stairs(KF_desired(7,:),'g--','LineWidth',.5)
ylabel('q_1','FontSize',12)
legend({'Actual','Measurements','Kalman Filter'},'FontSize',10)
title('Attitude Quaternion','FontSize',14)
axis([0 tmax/Ts -1 1])
subplot(4,1,2)
plot(actual(8,:),'k','LineWidth',.5)
hold on
plot(y(8,:),'b.')
stairs(x_hat_kk(8,:),'r-','LineWidth',1)
% stairs(KF_desired(8,:),'g--','LineWidth',.5)
ylabel('q_2','FontSize',12)
axis([0 tmax/Ts -1 1])
subplot(4,1,3)
plot(actual(9,:),'k','LineWidth',.5)
hold on
plot(y(9,:),'b.')
stairs(x_hat_kk(9,:),'r-','LineWidth',1)
% stairs(KF_desired(9,:),'g--','LineWidth',.5)
ylabel('q_3','FontSize',12)
axis([0 tmax/Ts -1 1])
subplot(4,1,4)
plot(actual(10,:),'k','LineWidth',.5)
hold on
plot(y(10,:),'b.')
stairs(x_hat_kk(10,:),'r-','LineWidth',1)
% stairs(KF_desired(10,:),'g--','LineWidth',.5)
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
% stairs(KF_desired(11,:),'g--','LineWidth',.5)
ylabel('\beta_x (rad/s)','FontSize',12)
legend({'Actual','Kalman Filter'},'FontSize',10)
title('Gyro Angular Rate Bias','FontSize',14)
axis([0 tmax/Ts -.25 .16])
subplot(3,1,2)
plot(actual(12,:),'k','LineWidth',.5)
hold on
stairs(x_hat_kk(12,:),'r-','LineWidth',1)
% stairs(KF_desired(12,:),'g--','LineWidth',.5)
ylabel('\beta_y (rad/s)','FontSize',12)
axis([0 tmax/Ts -.25 .16])
subplot(3,1,3)
plot(actual(13,:),'k','LineWidth',.5)
hold on
stairs(x_hat_kk(13,:),'r-','LineWidth',1)
% stairs(KF_desired(13,:),'g--','LineWidth',.5)
ylabel('\beta_z (rad/s)','FontSize',12)
axis([0 tmax/Ts -.25 .16])
xlabel('Time (s)','FontSize',12)
set(gcf,'Position',[488,100,560,700]) % x,y,width,height