function [ J ] = Jh( x, u, Ts )
%Jacobian of measurement equation Jh = dh/dt
%   x:      state
%   u:      input
%   Ts:     sample time
%   J:      Jacobian

a1 = 0;
b1 = 1;
c1 = 0.1;   % line: y=0.05;
a2 = 1;
b2 = 0;
c2 = 0.25;   % line: x=0.25

J = [a1/sqrt(a1^2+b1^2),    b1/sqrt(a1^2+b1^2), 0;
     a2/sqrt(a2^2+b2^2),    b2/sqrt(a2^2+b2^2), 0];
end

