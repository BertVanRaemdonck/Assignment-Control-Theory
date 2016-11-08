function unwrapped = cust_unwrap(values, n)
% CUST_UNWRAP  unwraps a series of values.
%   U = CUST_UNWRAP(A,n) unwraps a A with the elements of A between -2^(n-1) and
%   2^(n-1) -1

unwrapped = values;
for i = length(unwrapped)-1:-1:1    
    if sign(unwrapped(i))==1 && sign(unwrapped(i+1))==-1
        if unwrapped(i) > 2^(n-2)                          % Wrapping occurs when the value is over halfway between 0 and the maximal value
            unwrapped(i+1:end) = unwrapped(i+1:end) + 2^n;
        end
    end
    if sign(unwrapped(i))==-1 && sign(unwrapped(i+1))==1
        if unwrapped(i) < -2^(n-2)                         % Wrapping occurs when the value is over halfway between 0 and the minimal value
            unwrapped(i+1:end) = unwrapped(i+1:end) - 2^n;
        end
    end
end

end