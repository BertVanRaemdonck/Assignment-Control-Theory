function [ y ] = h( x, u, Ts )
% Measurement equation of system y(k) = h(x(k), u(k), Ts)
%   y:      state at index k
%   u:      input at index k
%   Ts:     sample time
%   y:      measurement at index k

a1 = 0;
b1 = 1;
c1 = 0.1;   % line: y=0.05;
a2 = 1;
b2 = 0;
c2 = 0.25;   % line: x=0.25

y =  [((a1*x(1))+(b1*x(2))-c1)/sqrt((a1^2)+(b1^2));
      ((a2*x(1))+(b2*x(2))-c2)/sqrt((a2^2)+(b2^2))];
end

