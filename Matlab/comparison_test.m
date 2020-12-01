%{
Author: Mark MacGillivray
Project: Dalhousie CubeSat
SubSystem: ADCS
Date: 2020-10-15

Description:
Compares two matrices by finding the difference between
each element of either array. If the relative error between all elements
of both matrices is less than the specified threshold, the two matrices
have equal values, and have the same order. These differences form a new
matrix called "error_matrix". This is printed to a file called
"error_matrix.txt".
%}

clc
clear all

%{
Opens .txt files containing output values to be compared. The files must
be located in the same folder as comparison_test. Change these two names to
the names of the matlab and C++ output files.
%}

matlab_output = importdata('s_hat_BF_matlab.txt'); 
cpp_output = importdata('s_hat_BF_cpp.txt');
threshold = 1e-9;

%The size of both matrices are checked for equality.
problem = false;
if size(matlab_output) == size(cpp_output)
   rows = size(matlab_output,1);
   cols = size(matlab_output,2);
    disp("dimensions: " + size(matlab_output,1) + "x" + size(matlab_output,2));
else
    disp("error: both files have different dimensions");
end

error_matrix = (abs(abs(matlab_output) - abs(cpp_output)))./abs(matlab_output);
max_error =  max(error_matrix, [], 'all');
%{
All elements of matlab and C++ outputs are checked for a difference larger
than a specific threshold number.
%}
for i = 1:rows
    for j = 1:cols
        if error_matrix(i,j) > threshold
            problem = true;
            disp("error above threshold at element " + i + ", " + j);
        else
        end
    end    
end
if ~problem
    % Print results.
    disp(newline + "outputs are equal"); 
    dlmwrite('error_matrix.txt', error_matrix, 'delimiter', ' ', 'precision', 15);
    disp("error matrix printed as error_matrix.txt");
else    
end
