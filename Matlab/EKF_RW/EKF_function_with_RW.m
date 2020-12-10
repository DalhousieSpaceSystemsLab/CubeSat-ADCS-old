function [x_hat_kk,P_kk] = EKF_function_with_gyro(x_hat_k_1k_1,P_k_1k_1,...
                                                 J,Jw,R,Q,Ts,u_k_1,u_RW_k_1,y_k)
% Extended Kalman Filter Algorithm with Reaction Wheels (RWs)
%
% INPUTS:
% =======
% x_hat_k_1k_1 = previous state estimate
% P_k_1k_1     = previous covariance estimate
% J            = inertia matrix of satellite (except for inertia of RW along spin axes) expressed in BF, kgm^2
% Jw           = inertia matrix of three RW aligned with body axes expressed in BF, kgm^2
% R            = measurement noise covariance matrix
% Q            = process noise covariance matrix
% Ts           = sample time (s)
% u_k_1        = previous external input torques applied to satellite expressed in BF, Nm
% u_RW_k_1     = previous input torques on RWs from motors, Nm
% y_k          = measurement vector
%
% OUTPUTS:
% ========
% x_hat_kk     = updated state estimate
% P_kk         = updated covariance estimate
%
% This software is provided "as is" and any expressed or implied 
% warrantees, including, but not limited to, the implied warranties
% of merchantability and fitness for a particular purpose are disclaimed.
% Dr. Robert Bauer shall not be liable for any direct, indirect,
% consequential, or other damages suffered by anyone resulting from this
% work or the use of the research results/data of this work.
%
%% State Prediction
% x_hat_k_1k_1 = [OMEGA_k_1k_1;
%                 w_k_1k_1;
%                 q_k_1k_1;
%                 q0_k_1k_1;
%                 beta_k_1k_1];
OMEGA_k_1k_1=x_hat_k_1k_1(1:3);
   w_k_1k_1 =x_hat_k_1k_1(4:6);
   q_k_1k_1 =x_hat_k_1k_1(7:9);
   q0_k_1k_1=x_hat_k_1k_1(10);
   
%State Prediction

%disp(-inv(J)*skew(w_k_1k_1)*(J*w_k_1k_1+Jw*OMEGA_k_1k_1) + inv(J)*u_k_1 - inv(J)*u_RW_k_1);

x_hat_kk_1 = x_hat_k_1k_1 + ...
    [ inv(Jw)*u_RW_k_1;
     -inv(J)*skew(w_k_1k_1)*(J*w_k_1k_1+Jw*OMEGA_k_1k_1) + inv(J)*u_k_1 - inv(J)*u_RW_k_1; 
     %note RW apply equal but opposite torque onto motors/satellite (-ve sign of torque on RW from motors above)
     -0.5*skew(w_k_1k_1)*q_k_1k_1 + 0.5*q0_k_1k_1*w_k_1k_1;
     -0.5*w_k_1k_1'*q_k_1k_1;
      zeros(3,1)]*Ts;
  
  
%% Covariance Prediction
dF1_OMEGA=-inv(J)*skew(w_k_1k_1)*Jw*Ts;
dF1_dw = eye(3)-inv(J)*(skew(w_k_1k_1)*J-skew(J*w_k_1k_1+Jw*OMEGA_k_1k_1))*Ts;

dF2_dw = 0.5*(skew(q_k_1k_1) + q0_k_1k_1*eye(3))*Ts;
dF2_dq = eye(3)-0.5*skew(w_k_1k_1)*Ts;
dF2_dq0= 0.5*w_k_1k_1*Ts;

dF3_dw = -0.5*q_k_1k_1'*Ts;
dF3_dq = -0.5*w_k_1k_1'*Ts;
dF3_dq0= 1;

F_k_1 = [   eye(3), zeros(3),  zeros(3), zeros(3,1),   zeros(3);
         dF1_OMEGA,   dF1_dw,  zeros(3), zeros(3,1),   zeros(3);
          zeros(3),   dF2_dw,    dF2_dq,    dF2_dq0,   zeros(3);
        zeros(1,3),   dF3_dw,    dF3_dq,    dF3_dq0, zeros(1,3);
          zeros(3), zeros(3),  zeros(3), zeros(3,1),     eye(3)];
     
L_k_1 = eye(13,13)*Ts;
%Covariance Prediction 
P_kk_1 = F_k_1*P_k_1k_1*F_k_1' + L_k_1*Q*L_k_1';

%% Innovation (Measurement Residual)
H=[  eye(3)   zeros(3)   zeros(3)   zeros(3,1) zeros(3);
   zeros(3)     eye(3)   zeros(3)   zeros(3,1)   eye(3);
   zeros(3)   zeros(3)     eye(3)   zeros(3,1) zeros(3);
   zeros(1,3) zeros(1,3) zeros(1,3) 1          zeros(1,3)];

y_tilda_k = y_k-H*x_hat_kk_1;

%% Innovation Covariance
S_k = H*P_kk_1*H'+R;

%% Kalman Gain
if isnan(cond(S_k)) %check invertibility of S_k
    disp('singularity in S_k')
    K_k = zeros(13,10); %if S_k cannot be inverted due to singularity
else
    K_k = P_kk_1*H'*inv(S_k);
end
%% State Update
x_hat_kk = x_hat_kk_1 + K_k*y_tilda_k;

q=x_hat_kk(7:10);
q = q/norm(q,2); %normalize quaterion estimate
x_hat_kk(7:10) = q;

%resulting estimate
%OMEGA_k= x_hat_kk(1:3);
%w_k    = x_hat_kk(4:6);
%q_k    = x_hat_kk(7:9);
%q0_k   = x_hat_kk(10);
%beta_k = x_hat_kk(11:13);

%% Covariance Update
P_kk = (eye(13) - K_k*H)*P_kk_1;
end


