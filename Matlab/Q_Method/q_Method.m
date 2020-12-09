function q_est = q_Method(b1,b2,b3,r1,r2)
% q-Method Attitude Determination Algorithm
% DEFINE INPUTS/OUTPUTS
% Input:
% ======
% uses two different vectors (eg. Sun and magnetic field) who's components
% are expressed in Body-Fixed and ECI frames:
% b1 = components of Sun vector expressed in Body-Fixed frame (from Diodes)
% b2 = components of magnetic field vector A expressed in Body-Fixed frame
%      (from magnetometer A)
% b3 = components of magnetic field vector B expressed in Body-Fixed frame
%      (from magnetometer B)
% r1 = components of Sun vector expressed in ECI frame
%      (from theory)
% r2 = components of magnetic field vector expressed in ECI frame
%      (from theory)
% NOTE: the above are column matrices
%       ie:  b1=[b1x,b1y,b1z]'
%
% Output:
% =======
%   q_est  = estimated quaternion orientation

% reference: Markley, F., Crassidis, J., 
% "Fundmentals of Spacecraft Attitude Determination and Control"
% Space Technology Library, Springer, 2014, page 184
%
% also based on Example 4.3 in Chris Hall's March 18 2003 Ch4 notes
% online: http://www.dept.aoe.vt.edu/~cdhall/courses/aoe4140/attde.pdf
%
% This software is for Dal CubeSat project internal use only and
% is a work-in-progress.
% This software is provided "as is" and any expressed or implied 
% warrantees, including, but not limited to, the implied warranties
% of merchantability and fitness for a particular purpose are disclaimed.
% Dr. Robert Bauer shall not be liable for any direct, indirect,
% consequential, or other damages suffered by anyone resulting from this
% work or the use of the research results/data of this work.
%%
w1=1; % weightimg matrix for first measurement (Sun vector)
w2=1; % weightimg matrix for second measurement (mag field vector A)
w3=1; % weightimg matrix for third measurement (mag field vector B)

B = w1*(b1*r1')+w2*(b2*r2')+w3*(b3*r2'); %if have two mag field measurements
S = B + B';
Z = [B(2,3)-B(3,2), B(3,1)-B(1,3), B(1,2)-B(2,1)]';
sigma = trace(B);

% q-Method:
% based on Example 4.3 in Chris Hall's March 18 2003 notes
% online: http://www.dept.aoe.vt.edu/~cdhall/courses/aoe4140/attde.pdf
% also summarized in Fundamentals of Spacecraft Attitude Determination
% on page 187, (Eq references refer to this text)
K = [S-sigma*eye(3,3),     Z;
                   Z', sigma]; %Eq 5.17
 
% the eigenvector corresponding to the largest eigenvalue of K is the
% least-squares optimal estimate of the attitude
[V,D]=eig(K); %calculate diagonal matrix D of eigenvalues and
              %matrix V whos columns are corresponding eigenvectors
D=diag(D);    %convert diagonal matrix D into a column vector of eigenvalues
[value,indx]=max(real(D)); %find index of the maximum eigenvalue
q_est = real(V(:,indx)); % use corresponding eigenvector as the 
                         % estimated attitude quaternion 
                         % in [xi yj zk w] format, Eq. 5.23
                
                         

