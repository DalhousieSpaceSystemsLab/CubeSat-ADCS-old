% compare IGRF_rjb() with Matlab's igrfmagm()for IGRF Generation 12
% which works between Jan 2015 to Dec 2019
clear all
close all
format long
% pick a random date, geodetic latitude, longitude and altitude: 
date_cur = [2016 2 27]; %Feb 27, 2016
lat_geodetic=30.166923849507349; %geodetic latitude (deg)
phi=23; %longitude (deg)
H=300;%altitude above Earth's surface (km)
[XYZ_rjb]=IGRF_rjb_V2(lat_geodetic,phi,H,date_cur)
[XYZ_Matlab]=igrfmagm(H*1000,lat_geodetic,phi,decyear(date_cur),12)

return
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matlab's function: igrfmagm()
%    INPUTS:
%    =======
%     HEIGHT :Scalar value in meters. 
%     LAT    :Scalar geodetic latitude in degrees where north latitude is 
%             positive and south latitude is negative.
%     LON    :Scalar geodetic longitude in degrees where east longitude 
%             is positive and  west longitude is negative.
%     DYEAR  :Scalar decimal year.  Decimal year is the desired year in 
%             a decimal format that includes any fraction of the year that has 
%             already passed.
%     GEN    :Scalar with the generation of the International Geomagnetic
%             Reference Field. Available options are 11 and 12. Default option
%             is 12.
% OUTPUTS:
% ========
%     XYZ    :Magnetic field vector in nanotesla (nT). Z is the vertical component (+ve down)
%     H      :Horizontal intensity in nanotesla (nT).
%     DEC    :Declination in degrees. (+ve east)
%     DIP    :Inclination in degrees. (+ve down)
%     F      :Total intensity in nanotesla (nT).
%
