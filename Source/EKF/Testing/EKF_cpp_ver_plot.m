
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

%% Define Constants
tmax = 100;% s, simulation time
Ts = 1;    % s, sample time


%% Plot Angular Velocity
figure(1)
subplot(3,1,1)
plot(desired(2,:),'k','LineWidth',.5)
hold on
plot(y(1,:),'b.')
stairs(x_hat_kk_cpp(1,:),'g-','LineWidth',1)
ylabel('\omega_x (rad/s)','FontSize',12)
legend({'Actual','Biased Measurements','Kalman Filter'},'FontSize',10)
title('Angular Velocity','FontSize',14)
axis([0 tmax/Ts -.16 .16])
subplot(3,1,2)
plot(desired(3,:),'k','LineWidth',.5)
hold on
plot(y(2,:),'b.')
stairs(x_hat_kk_cpp(2,:),'g-','LineWidth',1)
ylabel('\omega_y (rad/s)','FontSize',12)
axis([0 tmax/Ts -.16 .16])
subplot(3,1,3)
plot(desired(4,:),'k','LineWidth',.5)
hold on
plot(y(3,:),'b.')
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
stairs(x_hat_kk_cpp(4,:),'g-','LineWidth',1)
ylabel('q_1','FontSize',12)
legend({'Actual','Measurements','Kalman Filter'},'FontSize',10)
title('Attitude Quaternion','FontSize',14)
axis([0 tmax/Ts -1 1])
subplot(4,1,2)
plot(desired(6,:),'k','LineWidth',.5)
hold on
plot(y(5,:),'b.')
stairs(x_hat_kk_cpp(5,:),'g-','LineWidth',1)
ylabel('q_2','FontSize',12)
axis([0 tmax/Ts -1 1])
subplot(4,1,3)
plot(desired(7,:),'k','LineWidth',.5)
hold on
plot(y(6,:),'b.')
stairs(x_hat_kk_cpp(6,:),'g-','LineWidth',1)
ylabel('q_3','FontSize',12)
axis([0 tmax/Ts -1 1])
subplot(4,1,4)
plot(desired(8,:),'k','LineWidth',.5)
hold on
plot(y(7,:),'b.')
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
stairs(x_hat_kk_cpp(8,:),'g-','LineWidth',1)
ylabel('\beta_x (rad/s)','FontSize',12)
legend({'Actual','Kalman Filter'},'FontSize',10)
title('Gyro Angular Rate Bias','FontSize',14)
axis([0 tmax/Ts -.16 .16])
subplot(3,1,2)
plot(desired(10,:),'k','LineWidth',.5)
hold on
stairs(x_hat_kk_cpp(9,:),'g-','LineWidth',1)
ylabel('\beta_y (rad/s)','FontSize',12)
axis([0 tmax/Ts -.16 .16])
subplot(3,1,3)
plot(desired(11,:),'k','LineWidth',.5)
hold on
stairs(x_hat_kk_cpp(10,:),'g-','LineWidth',1)
ylabel('\beta_z (rad/s)','FontSize',12)
axis([0 tmax/Ts -.16 .16])
xlabel('Time (s)','FontSize',12)
set(gcf,'Position',[488,100,560,700]) % x,y,width,height