% saves IGRF_rjb()'s output for random dates in validation_dates.txt to
% validation_output_matlab.txt
clear all
close all
format long

lat_geodetic=30.166923849507349; %geodetic latitude (deg)
phi=23; %longitude (deg)
H=300;%altitude above Earth's surface (km)

% import input values from the text file: 
input = importdata('./Inputs/IGRF_validation_input_V2.txt');
[rows_in, cols_in] = size(input);

% import cpp output values from the text file: 
cpp_out = importdata('./Outputs/IGRF_validation_cpp.txt');
[rows_cpp, cols_cpp] = size(cpp_out);

output_file = fopen('./Outputs/IGRF_validation_matlab.txt','w');
fprintf(output_file,'lat_geodetic\tphi\tH\tYYYY\tMM\tDD\tbx\tby\tbz\terr_bx\terr_by\terr_bz\terr_bx_pr\terr_by_pr\terr_bz_pr\n');

for i = 1:rows_in
    loc_vec = [input(i, 1), input(i, 2), input(i, 3)];
    date_cur = [input(i, 4) input(i, 5) input(i, 6)];
    [XYZ_rjb]=IGRF_rjb_V3(loc_vec(1), loc_vec(2), loc_vec(3), date_cur);
    bx = num2str(XYZ_rjb(1),'%5.15f\n');
    by = num2str(XYZ_rjb(2),'%5.15f\n');
    bz = num2str(XYZ_rjb(3),'%5.15f\n');
    
    % calculate raw error
    err_bx = num2str(XYZ_rjb(1) - cpp_out(i, 7),'%5.15f\n');
    err_by = num2str(XYZ_rjb(2) - cpp_out(i, 8),'%5.15f\n');
    err_bz = num2str(XYZ_rjb(3) - cpp_out(i, 9),'%5.15f\n');
    
    % calculate absolute percentage error
    err_bx_pr = abs(XYZ_rjb(1) - cpp_out(i, 7));
    err_by_pr = abs(XYZ_rjb(2) - cpp_out(i, 8));
    err_bz_pr = abs(XYZ_rjb(3) - cpp_out(i, 9));
    err_bx_pr = num2str(abs(err_bx_pr * 100 / XYZ_rjb(1)),'%5.15f\n');
    err_by_pr = num2str(abs(err_by_pr * 100 / XYZ_rjb(2)),'%5.15f\n');
    err_bz_pr = num2str(abs(err_bz_pr * 100 / XYZ_rjb(3)),'%5.15f\n');
    fprintf(output_file,'%f\t%f\t%f\t%d\t%d\t%d\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n',loc_vec, date_cur, bx, by, bz, err_bx, err_by, err_bz, err_bx_pr, err_by_pr, err_bz_pr);
%     XYZ_rjb, err_bx, err_by, err_bz
%     loc_vec, date_cur
%     XYZ_rjb(1), XYZ_rjb(2), XYZ_rjb(3)
%     err_bx_pr, err_by_pr, err_bz_pr
end
fclose('all');

return
