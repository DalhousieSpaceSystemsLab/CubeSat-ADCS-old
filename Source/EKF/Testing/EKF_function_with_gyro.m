function [x_hat_kk,P_kk] = EKF_function_with_gyro(x_hat_k_1k_1,P_k_1k_1,...
                                                 J,R,Q,Ts,u_k_1,y_k)
%% Extended Kalman Filter (EKF) Algorithm

% This software is provided "as is" and any expressed or implied 
% warrantees, including, but not limited to, the implied warranties
% of merchantability and fitness for a particular purpose are disclaimed.
% Dr. Robert Bauer shall not be liable for any direct, indirect,
% consequential, or other damages suffered by anyone resulting from this
% work or the use of the research results/data of this work.

% Inputs:
% -------
% x_hat_k_1k_1 = previous state estimate 10x1, states consists of:
%                angular velocity (w)                     3x1
%                attitude quaternion (q vector portion)   3x1
%                attitude quaternion (q0 scalar portion)  1x1
%                gyro angular velocity bias (beta)        3x1
%     P_k_1k_1 = previous covariance estimate 10x10
%            J = inertia matrix of CubeSate 3x3, kgm^2
%            R = measurement noise covariance matrix 7x7
%            Q = process noise covariance matrix 10x10
%           Ts = measurement sample time, seconds
%        u_k_1 = input torques on CubeSat 3x1, Nm 
%                (from previous time step k-1)
%          y_k = measurement from current time step k, 7 measurements
%                1. wx CubeSat angular velocity relative to ECI expressed 
%                   in BF (x component) rad/s
%                2. wy CubeSat angular velocity relative to ECI expressed
%                   in BF (y component) rad/s
%                3. wz CubeSat angular velocity relative to ECI expressed
%                   in BF (z component) rad/s
%                4. q1 element 1 of vector portion of quaternion 
%                   representing rotation from ECI to BF
%                5. q2 element 2 of vector portion of quaternion 
%                   representing rotation from ECI to BF
%                6. q3 element 3 of vector portion of quaternion 
%                   representing rotation from ECI to BF
%                7. q0 scalar element quaternion 
%                   representing rotation from ECI to BF
% Outputs:
% --------
%     x_hat_kk = state estimate      10x1
%         P_kk = covariance estimate 10x10

%% State Prediction x_hat_kk_1 from previous estimate x_hat_k_1k_1 10x1
% x_hat_k_1k_1 = [w_k_1k_1;
%                 q_k_1k_1;
%                 q0_k_1k_1;
%                 beta_k_1k_1]; % 10x1

w_k_1k_1 =x_hat_k_1k_1(1:3);    % 3x1
q_k_1k_1 =x_hat_k_1k_1(4:6);    % 3x1
q0_k_1k_1=x_hat_k_1k_1(7);      % scalar

% State Prediction x_hat_kk_1 10x1
x_hat_kk_1 = x_hat_k_1k_1 + ...
    [-inv(J)*skew(w_k_1k_1)*(J*w_k_1k_1) + inv(J)*u_k_1;
     -0.5*skew(w_k_1k_1)*q_k_1k_1 + 0.5*q0_k_1k_1*w_k_1k_1;
     -0.5*w_k_1k_1'*q_k_1k_1;
      zeros(3,1)]*Ts;
%% Covariance Prediction P_kk_1 from previuos prediction P_k_1_k_1 10x10
dF1_dw = eye(3)-inv(J)*(skew(w_k_1k_1)*J-skew(J*w_k_1k_1))*Ts; % 3x3

dF2_dw = 0.5*(skew(q_k_1k_1) + q0_k_1k_1*eye(3))*Ts;           % 3x3
dF2_dq = eye(3)-0.5*skew(w_k_1k_1)*Ts;                         % 3x3
dF2_dq0= 0.5*w_k_1k_1*Ts;                                      % 3x1

dF3_dw = -0.5*q_k_1k_1'*Ts;                                    % 1x3
dF3_dq = -0.5*w_k_1k_1'*Ts;                                    % 1x3
dF3_dq0= 1;                                                    % 1x1

F_k_1 = [dF1_dw,  zeros(3), zeros(3,1),   zeros(3);
         dF2_dw,    dF2_dq,    dF2_dq0,   zeros(3);
         dF3_dw,    dF3_dq,    dF3_dq0, zeros(1,3);
         zeros(3),zeros(3), zeros(3,1),     eye(3)]; % 10x10
     
L_k_1 = eye(10,10)*Ts;                                       
% Covariance Prediction P_kk_1 10x10
P_kk_1 = F_k_1*P_k_1k_1*F_k_1' + L_k_1*Q*L_k_1';

%% Innovation (Measurement Residual) y_tilda_k 7x1
H=[eye(3)     zeros(3)   zeros(3,1) eye(3);
   zeros(3)   eye(3)     zeros(3,1) zeros(3);
   zeros(1,3) zeros(1,3) 1          zeros(1,3)];     % 7x10

y_tilda_k = y_k-H*x_hat_kk_1; % 7x1

%% Innovation Covariance S_k 7x7
S_k = H*P_kk_1*H'+R;

%% Kalman Gain K_k 10x7
if isnan(cond(S_k)) %check invertibility of S_k
    disp('singularity in S_k')
    K_k = zeros(10,7); % if S_k cannot be inverted due to singularity
else
    K_k = P_kk_1*H'*inv(S_k);
end
%% State Update x_hat_kk 10x1
x_hat_kk = x_hat_kk_1 + K_k*y_tilda_k;

q=x_hat_kk(4:7);   % isolate quaternion estimate
q = q/norm(q,2);   % normalize quaterion estimate
x_hat_kk(4:7) = q; % update x_hat_kk with normalized quaternion

% resulting state estimates
% w_k    = x_hat_kk(1:3);
% q_k    = x_hat_kk(4:6);
% q0_k   = x_hat_kk(7);
% beta_k = x_hat_kk(8:10);

%% Covariance Update P_kk 10x10
P_kk = (eye(10) - K_k*H)*P_kk_1;
end


