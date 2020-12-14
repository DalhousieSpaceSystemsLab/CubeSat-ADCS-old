function [XYZ]=IGRF_rjb_V3(lat_geodetic,phi,H,date_cur)
%% International Geomagnetic Reference Field (IGRF) Model for Earth
% this version developed by:
% Dr. Robert Bauer
% November 22, 2019
% Modified by Rutwij Makwana
% November 17, 2020
%
% This function uses the IGRF model to calculate Earth's magnetic field
% strength vector at a specified desired location
%
% INPUTS:
% =======
% lat_geodetic = geodetic latitude (deg) of desired location (geodetic coordinate)
%                values range between -90 and +90 deg (but not -90 or +90)
%          phi = longitude (deg) of desired location Note: longitude is the same for both
%                geodetic and geocentric spherical coordinates
%                values range between -180 and +180 deg
%            H = altitude above Earth's surface (km), which is perpendicular
%                to the surface of Earth (geodetic coordinate)
%                values range between -1km to 600km
%     date_cur = current time expressed as a vector [yyyy mm dd] such that
%                date_cur(1) = year,  eg. 2021
%                date_cur(2) = month, eg. 11
%                cate_cur(3) = day,   eg. 15
%                values range between [2020 1 1] to [2024 12 31]
% OUTPUTS:
% ========
%          XYZ = magnetic field strength vector (nanoTesla) expressed in 
%                North-East-Down (NED) coordinate frame
%                [North, East, Down]=[Bx,By,Bz]=XYZ
% NED coordinate frame's origin is located at the desired location (eg.
% location of satellite)
% North points tangential to Earth’s ellipsoidal surface towards the 
% polar North (+X pointing North)
% East points parallel to lines of latitude (+Y pointing East) 
% Down points antiparallel to Earth’s ellipsoidal 
% surface normal (+Z pointing down)
% Note: given Earth's ellipsoidal surface, the NED frame Z-axis does
% not pass through Earth's center
%
% the datafile '2020_IGRF.txt' was obtained from 
% https://www.ngdc.noaa.gov/IAGA/vmod/igrf.html
% Select the C software Windows version on this website and
% extract and find the IGRF13.COF file, copy and paste the 2020 data
%
% Ref A: "Mathematical modeling of Earth's magnetic field", Technical Note,
% Jeremy Davis, Virginia Tech, Blacksbury, VA 24061, May 12, 2004
% http://hanspeterschaub.info/Papers/UnderGradStudents/MagneticField.pdf
%
% Ref B: "The leap year problem", A. van Deursen
% https://pdfs.semanticscholar.org/c714/e1e6a2361e4227fa8ebbeb6a99e7c52111d8.pdf
%
% This software is for Dal CubeSat project internal use only and
% is a work-in-progress.
% This software is provided "as is" and any expressed or implied 
% warrantees, including, but not limited to, the implied warranties
% of merchantability and fitness for a particular purpose are disclaimed.
% Dr. Robert Bauer shall not be liable for any direct, indirect,
% consequential, or other damages suffered by anyone resulting from this
% work or the use of the research results/data of this work.
%
% Version 2: Nov 26, 2019
% -----------------------
% modified conversions from geodetic to geocentric spherical coordinates
% which still agrees with the original Fortran code but the equations
% used are now based on an actual reference document:
% Ref C: Algorithm Theoretical Basis Document, Christian-Albrechts-Universitata,
% Germany, 20 April 2015
% p7 and Matlab code at end p41
% https://earth.esa.int/documents/10174/1937037/GeoExplore_ATBD
% also modified the conversion from geocentric spherical back to geodetic
% which still agrees with the orgininal Fortran code but the equations
% used are now based on an actual reference document:
% Ref D: Spacecraft Attitude Determination and Control, James Wertz, Reidel Publishing
% Company, 1978
% also modified the iterations  to
% calculate Schmidt semi/quasi-normalized Legendre polynomial function
% so that the constant terms are brought out
% of the loop so there are fewer calculations required
%
% Version 3: Nov 17, 2020
% -----------------------
% Changed reference year to 2020 and added IGRF13 co-efficient file which
% is valid till 2025
%%

year_reference = 2020; % reference year for Magnetic field g and h coefficients
% calculate years plus fraction of a year since year_reference
% so that we can properly adjust the g and h parameters in the 
% Magnetic field to account for changes in the field wrt time
% obtain year, month and day from date_cur[] input vector
year = date_cur(1);
month= date_cur(2);
day  = date_cur(3);
% month-to-month cumulative days since Jan 1 of the current year 
% assuming no leap year
days=[0,31,59,90,120,151,181,212,243,273,304,334];
% From Ref B, it is a leapyear if:
% (the year can be evenly divided by 400)
% OR
% (the year can be evenly divided by 4 AND cannot be evenly divided by 100)
leap_year = mod(year,400) == 0 || ((mod(year,4) == 0) && mod(year,100) ~= 0);
% need to include the leap_year in the calculation only if the current date is into March
if month>2 
    leap_year_included=leap_year;  % current date is into March so
                                   % include whether or not it is a leap
                                   % year in the # days calculation below
else
    leap_year_included=0;          % current date is still in February so
                                   % don't need to include leap year 
                                   % in the # days calculation below
end
% calculate number of days so far in the current year
days_in_year = (days(month)-1)+day+leap_year_included;
% calculate the current year+fraction of current year, eg. 2016.25
% note need to divide by 366 if it is a leap year to calculate correct
% fraction of a year
year_current=year + days_in_year/(365+leap_year);
% calculate the time since the reference year as a years plus fraction of a year
% that can be used to linearly extrapolate the values for g and h in the
% magnetic field model
delta_t = year_current-year_reference; % this fractional year value is used to adjust g and h 
                                       % to account for changes in Magnetic 
                                       % Field model wrt time

% convert from geodetic to geocentric polar spherical coordinates: [theta,phi,r]
% r     (radial direction from Earth center),
% theta (inclination angle measured from ECEF +z axis)
% phi   (azimuth angle from ECEF +x axis)
%
% phi does not change, only theta_geodetic and altitude h change
a_ref = 6378.137;    % km, semimajor axis of reference model ellipsoide
f = 1/298.257223563; % wgs84Ellipsoid reference ellipsoid for Earth flattening
e2 = 2*f-f^2;        % eccentricity^2, Ref C, p6
% N
N = a_ref./sqrt(1-e2*sind(lat_geodetic).^2);
% from ellipsoidal to Cartesian
X = (N+H).*(cosd(lat_geodetic).*cosd(phi));
Y = (N+H).*(cosd(lat_geodetic).*sind(phi));
Z = (N*(1-e2)+H).*sind(lat_geodetic);
% from Cartesian to spherical
r = sqrt(X.^2+Y.^2+Z.^2);
lat_geocentric = asind(Z/r); % geocentric latitutde
% from geocentric latitude to geocentric co-latitude
theta_geocentric = 90-lat_geocentric; % geocentric co-latitude
% need sin and cos of geocentric co-latitude for 
% calculating Schmidt semi/quasi-normalized Legendre polynomial function
% later
costheta = cosd(theta_geocentric);
sintheta = sind(theta_geocentric);

%%
a = 6371.2;  % km, magnetic spherical reference radius (Earth radius)
n_max = 13;  % maximum degree, n<=13
m_max = z21(n_max); % 0-n_max, maximum degree (note Matlab not zero-based)

g = zeros(n_max,m_max); % IGRF g coefficient nanoTesla (nT)
h = zeros(n_max,m_max); % IGRF h coefficient nanoTesla (nT)

% SV = Secular Variation to extrapolate 2015 data (nT/year)
SVg = zeros(n_max,m_max); % SV for g (nT/year)
SVh = zeros(n_max,m_max); % SV for h (nT/year)

% import parameters for IGRF 2015 model
tmp = importdata('2020_IGRF.txt');

% Coefficient Text File has 6 columns corresponding to:
% 1 2  3  4   5     6
% n m  g  h SVg   SVh
%     nT nT nT/yr nT/yr
% SVg = Secular Variation to extrapolate 2015 data
tmp1=str2double(string(tmp.textdata(:,1:6))); %store data in temporary variable
[rows,cols]=size(tmp1);

for i = 1:rows
  % g(tmp1(i,1),z21(tmp1(i,2)))=tmp1(i,3);  % g 1-indexed on m, for example:
  %                                         % gm_n=g(n,z21(m))=g(5,3)=g2_5 (if m=2,n=5)
  % h(tmp1(i,1),z21(tmp1(i,2)))=tmp1(i,4);  % h 1-indexed on m
  % SVg(tmp1(i,1),z21(tmp1(i,2)))=tmp1(i,5);% SV on g, 1-indexed on m
  % SVh(tmp1(i,1),z21(tmp1(i,2)))=tmp1(i,6);% SV on h, 1-indexed on m
  
  % adjust g and h to current time using t*SV values
  % g                        =g        +delta_t*SVg
  g(tmp1(i,1),z21(tmp1(i,2)))=tmp1(i,3)+delta_t*tmp1(i,5);
  % h                        =h        +delta_t*SVh
  h(tmp1(i,1),z21(tmp1(i,2)))=tmp1(i,4)+delta_t*tmp1(i,6);
end

% g and h are now arrays that can be referenced with n and m such that
% g(n,z21(m))=gm_n, (m can be 0, so need z21(m) because Matlab is 1-indexed)
% similarly for h, h(n,z21(m))=hm_n

% Calculate magnetic field strength vector B expressed in geocentric
% spherical coordinates:
% theta (inclination angle measured from ECEF +z axis)
% phi   (azimuth angle from ECEF +x axis)
% r     (radial direction from Earth center),
% B = [Btheta,Bphi,Br]
% 
% initialize magnetic field
Btheta=0;
Bphi  =0;
Br    =0;
% begin nested loop for n and m
for n=1:n_max
    Btheta_sum = 0;
    Bphi_sum = 0;
    Br_sum = 0;
    for m=0:n
        % calculate Schmidt semi/quasi-normalized Legendre polynomial function 
        % of degree n and order m, along with its partial derivative
        [Pm_n,dPm_n]=calc_legendre(n,m,costheta,sintheta);
        % calculate magnetic fields from equations from Ref A
        Btheta_sum = Btheta_sum - dPm_n*...
                 (g(n,z21(m))*cosd(m*phi)+h(n,z21(m))*sind(m*phi));  % Eq 3b Ref A
        Bphi_sum   = Bphi_sum   - m*Pm_n*...
                 (-g(n,z21(m))*sind(m*phi)+h(n,z21(m))*cosd(m*phi)); % Eq 3c Ref A
        Br_sum     = Br_sum     + Pm_n*...
                 (g(n,z21(m))*cosd(m*phi)+h(n,z21(m))*sind(m*phi));  % Eq 3a Ref A 
    end
    Btheta = Btheta + ((a/r)^(n+2))*Btheta_sum;          % Eq 3b Ref A
    Bphi   = Bphi   + 1/sintheta*((a/r)^(n+2))*Bphi_sum; % Eq 3c Ref A
    Br     = Br     + ((a/r)^(n+2))*(n+1)*Br_sum;        % Eq 3a Ref A
end

% Convert back to geodetic coordinates
% Based on Ref D p782 Eq H-13 
cd = cosd(lat_geodetic - lat_geocentric); % lat_geocentric = 90-theta_geocentric
                                          % where lat_geocentric is delta
                                          % and theta_geocentric is the
                                          % geocentric co-latitude which is 'theta' in Ref D p782
sd = sind(lat_geodetic - lat_geocentric);
Bphi_geodetic   = Bphi;
Btheta_geodetic = Btheta*cd+Br*sd;
Br_geodetic     = Br*cd-Btheta*sd;

% Convert to NED: (North, East, Down) (Bx,By,Bz)
Bx=-Btheta_geodetic; % North is opposite direction to co-latitude theta
By= Bphi_geodetic;   % phi_geodetic is already East
Bz=-Br_geodetic;     % Down is opposite direction to radial direction r
XYZ=[Bx,By,Bz];      % output of function