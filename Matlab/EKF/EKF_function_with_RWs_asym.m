function [x_hat_kk,P_kk,wsi_kk] = EKF_function_with_RWs_asym(J,Jnew_inv,Is,...
    a1,a2,a3,R,Q,Ts,g_k_1,gai_k_1,y_k,x_hat_k_1,P_k_1)
% EKF function for the case where the CubeSat inertia matrix does not need 
% to be diagonal and the reaction wheel rotors do not need to be located 
% at the center of mass (COM)
%
% INPUTS:
% J         = inertia matrix of entire system about O (at COM) taken wrt BF axes, kgm^2
% Jnew_inv  = inverse of portion of J used in angular acceleration equation,1/kgm^2
% Is        = scalar inertia value of RW rotor about the COM of the rotor 
%             taken wrt to its spin axis, kgm^2
% ai        = spin axis direction (unit vector) of RWi expressed in BF, i=1,2,3
% R         = measurement noise covariance matrix
% Q         = process noise covariance matrix
% Ts        = sample time (s)
% g_k_1     = previous (k-1) external input torques applied to satellite expressed in BF, Nm
% gai_k_1   = scalar previous (k-1) input axial torque on RWi from motor i, Nm
% y_k       = current (k) measurement vector
% x_hat_k_1 = previous (k-1) state estimate
% P_k_1     = previous (k-1) covariance estimate

% OUTPUTS:
% x_hat_kk  = updated(kk) state estimate
% P_kk      = updated (kk) covariance estimate
% wsi_kk    = updated scalar relative angular speed of RWi wrt satellite 
%             about spin axis ai, i=1,2,3  (RW speed) rad/s

% This software is provided "as is" and any expressed or implied 
% warrantees, including, but not limited to, the implied warranties
% of merchantability and fitness for a particular purpose are disclaimed.
% Dr. Robert Bauer shall not be liable for any direct, indirect,
% consequential, or other damages suffered by anyone resulting from this
% work or the use of the research results/data of this work.
%% State Prediction
% x_hat_k_1 = [  ha1_k_1; %hai_k_1 is previous component along rotor spin  
%                ha2_k_1; %axis of absolute angular momentum of RWi about COM of Wi
%                ha3_k_1;
%                  w_k_1; %previous angular velocity of BF wrt to ECI expressed in BF
%                  q_k_1; %previous vector-portion of quaternion
%                 q0_k_1];%previous scalar-portion of quaternion
  ha1_k_1 = x_hat_k_1(1);
  ha2_k_1 = x_hat_k_1(2);
  ha3_k_1 = x_hat_k_1(3);
    w_k_1 = x_hat_k_1(4:6);
    q_k_1 = x_hat_k_1(7:9);
   q0_k_1 = x_hat_k_1(10);

%Convert RWi angular momentum to RWi angular velocity
ws1_k_1 = ha1_k_1/Is-a1'*w_k_1;
ws2_k_1 = ha2_k_1/Is-a2'*w_k_1;
ws3_k_1 = ha3_k_1/Is-a3'*w_k_1;

%separate RWi torques
ga1_k_1 = gai_k_1(1);
ga2_k_1 = gai_k_1(2);
ga3_k_1 = gai_k_1(3);

%State Prediction
x_hat_k_k_1 = x_hat_k_1 + [ ga1_k_1;
                            ga2_k_1;
                            ga3_k_1;
                            Jnew_inv*(-skew(w_k_1)*(J*w_k_1+a1*Is*ws1_k_1+...
                               a2*Is*ws2_k_1+a3*Is*ws3_k_1)-...
                               (a1*ga1_k_1+a2*ga2_k_1+a3*ga3_k_1)+g_k_1);
                           -0.5*skew(w_k_1)*q_k_1 + 0.5*q0_k_1*w_k_1;
                           -0.5*w_k_1'*q_k_1;
                           zeros(3,1)]*Ts;

%% Covariance Prediction
dF1_dw = eye(3)-Jnew_inv*(skew(w_k_1)*J-skew(J*w_k_1+a1*Is*ws1_k_1+...
    a2*Is*ws2_k_1 + a3*Is*ws3_k_1))*Ts;

dF2_dw = 0.5*(skew(q_k_1) + q0_k_1*eye(3))*Ts;
dF2_dq = eye(3)-0.5*skew(w_k_1)*Ts;
dF2_dq0= 0.5*w_k_1*Ts;

dF3_dw = -0.5*q_k_1'*Ts;
dF3_dq = -0.5*w_k_1'*Ts;
dF3_dq0= 1;

F_k_1=[1          0          0          zeros(1,3) zeros(1,3) 0          zeros(1,3);
       0          1          0          zeros(1,3) zeros(1,3) 0          zeros(1,3);
       0          0          1          zeros(1,3) zeros(1,3) 0          zeros(1,3);
       zeros(3,1) zeros(3,1) zeros(3,1) dF1_dw     zeros(3)   zeros(3,1) zeros(3);
       zeros(3,1) zeros(3,1) zeros(3,1) dF2_dw     dF2_dq     dF2_dq0    zeros(3);
       0          0          0          dF3_dw     dF3_dq     dF3_dq0    zeros(1,3);
       zeros(3,1) zeros(3,1) zeros(3,1) zeros(3)   zeros(3)   zeros(3,1) eye(3)];
     
L_k_1 = eye(13,13)*Ts;
%Covariance Prediction 
P_k_k_1 = F_k_1*P_k_1*F_k_1' + L_k_1*Q*L_k_1';

%% Innovation
H=[1/Is       0          0         -a1'        zeros(1,3) 0          zeros(1,3);
   0          1/Is       0         -a2'        zeros(1,3) 0          zeros(1,3);
   0          0          1/Is      -a3'        zeros(1,3) 0          zeros(1,3);
   zeros(3,1) zeros(3,1) zeros(3,1) eye(3)     zeros(3)   zeros(3,1) eye(3);
   zeros(3,1) zeros(3,1) zeros(3,1) zeros(3)   eye(3)     zeros(3,1) zeros(3);
   0          0          0          zeros(1,3) zeros(1,3) 1          zeros(1,3)];

y_tilda_k = y_k-H*x_hat_k_k_1;

%% Innovation Covariance
S_k = H*P_k_k_1*H'+R;

% Kalman Gain
if isnan(cond(S_k))
    disp('singularity in S_k')
    K_k = zeros(13,10); %if S_k cannot be inverted due to singularity
else
    K_k = P_k_k_1*H'*inv(S_k);
end
%% State Update
x_hat_kk = x_hat_k_k_1 + K_k*y_tilda_k;

q=x_hat_kk(7:10);
q = q/norm(q,2); %normalize quaterion estimate

x_hat_kk(7:10) = q;

%resulting estimate
ha1_kk  = x_hat_kk(1);
ha2_kk  = x_hat_kk(2);
ha3_kk  = x_hat_kk(3);
w_kk    = x_hat_kk(4:6);

%extract predicted angular velocity wsi_kk from predicted RWi angular momentum
ws1_kk = ha1_kk/Is-a1'*w_kk;
ws2_kk = ha2_kk/Is-a2'*w_kk;
ws3_kk = ha3_kk/Is-a3'*w_kk;

wsi_kk = [ws1_kk;
          ws2_kk;
          ws3_kk];
%% Covariance Update
P_kk = (eye(13) - K_k*H)*P_k_k_1;
end