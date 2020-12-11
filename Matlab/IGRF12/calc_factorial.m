function [result]=calc_factorial(x)
% this function calculates the factorial of x (x!)
if x == 0;
    result = 1; % 0! = 1
else
    result = x;
    for i = x-1:-1:2  % loop through to obtain x*(x-1)*(x-2)*...*2
        result = result*i;
    end
end
