function [ J ] = Jf( x, u, Ts )
%Jacobian of state equation Jf = df/dt
%   x:      state
%   u:      input
%   Ts:     sample time
%   J:      Jacobian

J = [1      0       -Ts*u(1)*sin(x(3));
     0      1       Ts*u(1)*cos(x(3));
     0      0       1];
end

