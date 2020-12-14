% compare IGRF_rjb_V3() with Matlab's igrfmagm()for IGRF Generation 13
% which works between Jan 2020 to Dec 2024
clear all
close all
format long
% pick a random date, geodetic latitude, longitude and altitude: 
date_cur = [2021 11 15]; %Nov 15, 2021
lat_geodetic=44.636682486410656; %geodetic latitude (deg)
phi=-63.59206788292759; %longitude (deg)
H=300;%altitude above Earth's surface (km)
[XYZ_rjb]=IGRF_rjb_V3(lat_geodetic,phi,H,date_cur)
[XYZ_Matlab]=igrfmagm(H*1000,lat_geodetic,phi,decyear(date_cur),13)

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
%             Reference Field. Available options are 11, 12 and 13. Default option
%             is 13.
% OUTPUTS:
% ========
%     XYZ    :Magnetic field vector in nanotesla (nT). Z is the vertical component (+ve down)
%     H      :Horizontal intensity in nanotesla (nT).
%     DEC    :Declination in degrees. (+ve east)
%     DIP    :Inclination in degrees. (+ve down)
%     F      :Total intensity in nanotesla (nT).
%
