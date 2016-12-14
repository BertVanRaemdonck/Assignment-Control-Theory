function [ x_next ] = f( x, u, Ts )
% Discrete differential equation of system: x(k+1) = f(x(k), u(k))
%   x:      state at index k
%   u:      input at index k
%   Ts:     sample time
%   x_next: state at index k+1

x_next = [x(1)+(Ts*cos(x(3))*u(1));
          x(2)+(Ts*sin(x(3))*u(1));
          x(3)+(Ts*u(2))];

end

