function [Pm_n_bar, dPm_n_bar]=calc_legendre(n_des,m_des,costheta,sintheta)
% calculate Schmidt semi/quasi-normalized Legendre polynomial function 
% of degree n and order m: Pm_n_bar(costheta) 
% and partial derivative wrt costheta: dPm_n_bar(costheta)
%
% Reference: Fundamentals of Spacecraft Attitude Determination
% Markley and Crassidis, Springer, 2014
% p384
% NOTE: THERE ARE TWO SIGN CONVENTIONS
% to equate the results of this function with Matlab's legendre()
% function, you need to compare Pm_n_FROM_MATLAB = (-1)^m *Pm_n_FROM_THIS FUNCTION
%
% refer to my .pdf notes for derivation of recursive form of partial derivatives
% of Pm_n_bar wrt theta

% INPUTS:
% =======
% n_des    = desired degree to calculate   (0 indexed)
% m_des    = desired order to calculate    (0 indexed)
% costheta = cosine of geocentric latitude (degrees)
% sintheta = sine of geocentric latitude   (degrees)

% OUTPUTS:
% ========
% Pm_n_bar = calculate Schmidt semi-normalized legendre function
%dPm_n_bar = partial derivative of Pm_n_bar wrt theta

% use z21 (zero-based to one-based) so we can work with 0-based indices
 P=zeros(z21(n_des),z21(m_des));  %  Pm_n
dP=zeros(z21(n_des),z21(m_des));  % dPm_n

% define initial Legendre functions used in recursive formulation
P(z21(0),z21(0)) = 1;            % n=0,m=0 P0_0 (one-indexed in Matlab)
P(z21(1),z21(0)) = costheta;     % n=1,m=0 P0_1 (one-indexed in Matlab)
P(z21(1),z21(1)) = sintheta;     % n=1,m=1 P1_1 (one-indexed in Matlab)

% define initial partial derivatives of Legendre functions used in recursive formulation
dP(z21(0),z21(0)) =  0;          % n=0,m=0 dP0_0 (one-indexed in Matlab)
dP(z21(1),z21(0)) = -sintheta;   % n=1,m=0 dP0_1 (one-indexed in Matlab)
dP(z21(1),z21(1)) =  costheta;   % n=1,m=1 dP1_1 (one-indexed in Matlab)

if n_des == 0 && m_des == 0      
    Pm_n = 1;         %  P0_0  Pm_n
   dPm_n = 0;         % dP0_0 dPm_n
elseif n_des == 1 && m_des == 0  
    Pm_n =  costheta; %  P0_1  Pm_n
   dPm_n = -sintheta; % dP0_1 dPm_n
elseif n_des == 1 && m_des == 1  
    Pm_n =  sintheta; %  P1_1  Pm_n
   dPm_n =  costheta; % dP1_1 dPm_n
else
    % need to calculate Pm_n and dPm_n using recursive formulation
    for n=2:n_des
        m=0;
        while m<=m_des
            if m == 0
                [m,n]; % 
                P(z21(n),z21(m)) = 1/n*((2*n-1)*costheta*P(z21(n-1),z21(m))-...
                    (n-1)*P(z21(n-2),z21(m))); % Eq 10.93a
                dP(z21(n),z21(m))= 1/n*((2*n-1)*(costheta*dP(z21(n-1),z21(m))-...
                    sintheta*P(z21(n-1),z21(m)))-...
                    (n-1)*dP(z21(n-2),z21(m))); % my derivation Eq RJB 1
            elseif m == n
                [n,m]; 
                P(z21(n),z21(m)) = (2*n-1)*sintheta*P(z21(n-1),z21(m-1)); % Eq 10.93c
                dP(z21(n),z21(m)) = (2*n-1)*(sintheta*dP(z21(n-1),z21(m-1))+...
                    costheta*P(z21(n-1),z21(m-1))); % my derivation Eq RJB 3
            else % 0 < m < n
                [n,m];
                P(z21(n),z21(m)) = costheta*P(z21(n-1),z21(m))+(n+m-1)*...
                    sintheta*P(z21(n-1),z21(m-1)); % Eq 10.93b
                dP(z21(n),z21(m)) = costheta*dP(z21(n-1),z21(m))-...
                    sintheta*P(z21(n-1),z21(m))+(n+m-1)*...
                    (sintheta*dP(z21(n-1),z21(m-1))+...
                    costheta*P(z21(n-1),z21(m-1))); % my derivation Eq RJB 2
            end
            m=m+1;
        end
        
    end
    Pm_n =  P(z21(n_des),z21(m_des)); % final result for  Pm_n
   dPm_n = dP(z21(n_des),z21(m_des)); % final result for dPm_n
end

% calculate corresponding Schmidt semi-normalized function using Eq 11.2
% Kronecker delta for delta_0m:
if m_des == 0
    delta = 1;
else
    delta = 0;
end
Pm_n_bar = sqrt((2-delta)*calc_factorial(n_des-m_des)/...
    calc_factorial(n_des+m_des))*Pm_n;  % Eq 11.2
dPm_n_bar = sqrt((2-delta)*calc_factorial(n_des-m_des)/...
    calc_factorial(n_des+m_des))*dPm_n; % Eq 11.2

