% saves IGRF_rjb()'s output for random dates in validation_dates.txt to
% validation_output_matlab_nocompare.txt
clear all
close all
format long

lat_geodetic=30.166923849507349; %geodetic latitude (deg)
phi=23; %longitude (deg)
H=300;%altitude above Earth's surface (km)

% import input values from the text file: 
input = importdata('../Inputs/IGRF_validation_input_V2.txt');
[rows_in, cols_in] = size(input);

output_file = fopen('../Outputs/validation_output_matlab_nocompare.txt','w');
% fprintf(output_file,'lat_geodetic\tphi\tH\tYYYY\tMM\tDD\tbx\tby\tbz\n');

for i = 1:rows_in
    loc_vec = [input(i, 1), input(i, 2), input(i, 3)];
    date_cur = [input(i, 4) input(i, 5) input(i, 6)];
    [XYZ_rjb]=IGRF_rjb_V3(loc_vec(1), loc_vec(2), loc_vec(3), date_cur);
    geo = num2str(input(i, 1),'%5.15f\n');
    ph = num2str(input(i, 2),'%5.15f\n');
    h = num2str(input(i, 3),'%5.15f\n');
    bx = num2str(XYZ_rjb(1),'%5.15f\n');
    by = num2str(XYZ_rjb(2),'%5.15f\n');
    bz = num2str(XYZ_rjb(3),'%5.15f\n');

    fprintf(output_file,'%s\t%s\t%s\t%d\t%d\t%d\t%s\t%s\t%s\n', geo, ph, h, date_cur, bx, by, bz);
end
fclose('all');

return