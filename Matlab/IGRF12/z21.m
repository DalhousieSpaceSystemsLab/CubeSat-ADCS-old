function [indx_one_based]=z21(indx_zero_based)
% Matlab uses one-based indexing so this function
% allows me to use zero-based indexing
% and the function adds 1 to my zero-based index to 
% enable Matlab to internally store/access the data in vectors/matrices
indx_one_based = indx_zero_based+1;
end