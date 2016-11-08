function d = central_diff(values, t)
% CENTRAL_DIFF  calculates the central difference for a series of values
%   d = CENTRAL_DIFF(values, t) gives the central difference approximation
%   of the derivative of values to t. The first and last value of d are
%   calculated with respectively a forward and backward difference so that
%   d has the same size as values.

d = zeros(size(values));

if length(values) >= 2
    d(1) = (values(2)-values(1))/(t(2)-t(1));               % forward difference
    d(end) = (values(end)-values(end-1))/(t(end)-t(end-1)); % backward difference
end
if length(values) >= 3
    for i = 2:length(values)-1
        d(i) = (values(i+1)-values(i-1))/(t(i+1)-t(i-1));   % central difference
    end
end

end