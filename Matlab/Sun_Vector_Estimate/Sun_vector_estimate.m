function s_hat_BF = Sun_vector_estimate(H,y) 
% Template for C language code to
% Calculate Sun vector estimate with a CubeSat having 
% 18 Photo Diode Sun Sensors

% DEFINE INPUTS/OUTPUTS
% Inputs:
% =======
% s=given 3x1 Sun vector direction expressed in BF
% H = 18 row concatenation of 1x3 normal vectors of Sun sensors expressed
%     in BF creating an 18x3 matrix
% y = 18 Sun raw sensor intensity measurements (values between 0 and 1)
%     creating an 18x1 vector

% Output:
% =======
% s_hat_BF = estimated Sun vector in BF

% based on Eq4.8 p59 in "Satellite Attitude Determination with Low-Cost
% Sensors" by John C. Springmann, Doctoral dissertation, University of
% Michigan, 2013
% https://deepblue.lib.umich.edu/handle/2027.42/102312

% This software is for Dal CubeSat project internal use only.
% Dr. Robert Bauer shall not be liable for any direct, indirect, 
% consequential, or other damages suffered by anyone resulting from this 
% work or the use of the research results/data of this work.

%% ESTIMATE SUN VECTOR
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% NEED TO CODE IN C THE FOLLOWING ALGORITHM
%%% 1.go through the vector of Sun intensity readings "y" and only 
%%%   keep those elements in "y" that exceed a threshold value 
%%%   (to avoid ambiguities from Earth Albedo)
%%% 2.check to ensure "y" has at least 3 elements in it (if not then
%%%   cannot estimate Sun vector)
%%% 3. assemble corresponding "H" matrix of normal vectors
%%% 4. check that H'*H is invertible
%%% 5. evaluate s_hat_BF = inv(H'*H)*H'*y, where s_hat_BF is the
%%%    estimated Sun vector in Body-Fixed Frame of reference.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

threshold = 0.5; % lower-bound sensor threshold value
% set the lower-bound sensor threshold value to be 0.5 because 
% the dot product of unit Sun vector along the
% edge of the FOV with the vertical (normal) direction is 0.5
% ie. 0.5 is the component of the unit Sun vector shown below
% projecte onto the normal vector:
%
%     normal vector
%         |
%       _ |____
%       | |   /-unit Sun vector
%      0.5|  /
%       | | /\ cone_angle=30deg defining Field Of View (FOV)
%_______|_|/_/_________

% if any sensor values are less than the threshold of 0.5
% then measurement is either due to Sun being outside
% the FOV or Earth's albedo radiation is being sensed
sens=find(y>threshold); % determine valid readings 
                        % "sens" contains the sensor number that can
                        % see Sun
[r,c]=size(sens); % determine how many (r) can see Sun

if r<3  % need at least 3 sensors that can see Sun
    disp('Need at least 3 sensors that can see Sun to estimate Sun vector')
    s_hat_BF=[0 0 0]'; % cannot determine Sun vector
else
    y=y(sens,:); % remove sensors that cannot see Sun
    H=H(sens,:); % normal vectors of each Sun sensor in BF that can see Sun
    % estimate new Sun vector
    %    y     = H    * s
    % [I1_I0;    [n1;
    %  I2_I0;  =  n2; * s
    %  I3_I0]     n3]
    % IMPORTANT: Check that matrix can be inverted
    if rcond(H'*H) > 1E-6 % if rcond(H'*H) is badly conditioned it is near 
                         % EPS and we cannot invert it
        % s_hat_BF = pinv(H)*y; %if don't know eta
        % or, equivalently:
        s_hat_BF = inv(H'*H)*H'*y; % this is how we will likely implement 
                                   % pinv()
    else
        s_hat_BF=[0 0 0]'; % cannot determine Sun vector
    end
end
