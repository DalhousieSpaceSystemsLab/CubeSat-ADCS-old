% The transformation quaternion between ECI and Nadir-Pointing (NP)
% reference frames is derived from the ECI - Perifocal - NP rotation 
% sequence below and the final rotation matrix is converted into a quaternion.
% Functions for principal rotations C1, C2, and C3 are found at the bottom
% of this script.

% Initial values for orbital elements are:
d2r = pi/180; % degree to rad conversion
omega = d2r*231.7821; %  rad, argument of perigee
RAAN = d2r*257.8729; % rad, right ascension of the ascending node
i = d2r*51.6413; % rad, inclination
% True anomaly %%%%%%%%%%%% Choose any value 0 < tano < 2*pi %%%%%%%%%%%%
tano = d2r*30;  %rad, true anomaly

% ECI to Perifocal rotation matrix, making use of orbital elements:
%   omega = argument of perigee (rad)
%   i = inclination (rad)
%   RAAN = right ascension of the ascending node (rad)
C_PI = C3(omega)*C1(i)*C3(RAAN);

% Perifocal to NP. Both frames lie in the orbital plane, perifocal is an
% inertial coordinate system while NP follows the satellite. The rotation 
% is a 3-2-3 sequence through true anomaly, -90, and 90, respectively. 
%   tano = true anomaly (rad) 
C_NPP = C3(pi/2)*C2(-pi/2)*C3(tano);

% Combining the rotation matrices:
C_NPI = C_NPP*C_PI;

% The calculation of this rotation matrix requires an orbit propagator
% implemented elsewhere from which true anomaly is derived. The orbital
% elements can be considered constant across one or several orbits, but
% will need to be updated periodically. This can be done independent of an
% orbit propagator (using gps values & updating on same interval?).

% Converting the matrix into a quaternion is a reasonably involved
% progress, cite: http://www.tu-berlin.de/fileadmin/fg169/miscellaneous/Quaternions.pdf

% Reassigning elements of the 3x3 matrix for ease of computation:
%                   | t11 t12 t13 |
%               C = | t21 t22 t23 |
%                   | t31 t31 t33 |
t11 = C_NPI(1,1); t12 = C_NPI(1,2); t13 = C_NPI(1,3);
t21 = C_NPI(2,1); t22 = C_NPI(2,2); t23 = C_NPI(2,3);
t31 = C_NPI(3,1); t32 = C_NPI(3,2); t33 = C_NPI(3,3);

% Quaternions are ordered scalar|vector, i.e.
%        | qs |
%        | qx |
%  q  =  | qy |
%        | qz |

% First pass:
qq = [sqrt(0.25*(1+t11+t22+t33));
      sqrt(0.25*(1+t11-t22-t33));
      sqrt(0.25*(1-t11+t22-t33));
      sqrt(0.25*(1-t11-t22+t33))];
% Find maximum element and recompute
[val, idx] = max(qq); % finds maximum element of qq and its index
if idx == 1 % qs is max
    qs = qq(1); % original
    qx = (t32-t23)/(4*qs);
    qy = (t13-t31)/(4*qs);
    qz = (t21-t12)/(4*qs);
elseif idx == 2 % qx is max
    qx = qq(2);  % original
    qs = (t32-t23)/(4*qx);
    qy = (t21+t12)/(4*qx);
    qz = (t13+t31)/(4*qx);
elseif idx == 3 % qy is max
    qy = qq(3);  % original
    qs = (t13-t31)/(4*qy);
    qx = (t21+t12)/(4*qy);
    qz = (t32+t23)/(4*qy);
elseif idx == 4 % qz is max
    qz = qq(4); % original
    qs = (t21-t12)/(4*qz);
    qx = (t13+t31)/(4*qz);
    qy = (t32+t23)/(4*qz);
end

% Build final quaternion from newly computed values:
q_NPI = [qs qx qy qz];

% Check this quaternion versus that obtained using matlab's built in
% conversion with function q_check = dcm2quat(C_NPI);
% Quaternions are not unique and the values may differ, plotting the vector
% components only should show them as coincident.

%Prints results to .txt file.
dlmwrite('q_NPI_matlab.txt', q_NPI, 'delimiter', ' ', 'precision', 15);
function C = C1(x)
% Places scalar angle x (rad) into standard rotation matrix about x-axis
C = [ 1      0      0 ;
      0  cos(x) sin(x);
      0 -sin(x) cos(x) ]; 
end

function C = C2(y)
% Places scalar angle y (rad) into standard rotation matrix about y-axis
C = [  cos(y) 0 -sin(y);
           0  1      0 ;
       sin(y) 0  cos(y) ];
end

function C = C3(z)
% Places scalar angle z (rad) into standard rotation matrix about z-axis
C = [  cos(z) sin(z) 0;
      -sin(z) cos(z) 0;
            0     0  1 ];
end