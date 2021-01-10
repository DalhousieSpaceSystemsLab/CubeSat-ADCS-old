function result=skew(x)
    % calculate skew-symmetric matrix of vector x
    result = [0   ,-x(3), x(2);
              x(3), 0   ,-x(1);
             -x(2), x(1),   0];
end