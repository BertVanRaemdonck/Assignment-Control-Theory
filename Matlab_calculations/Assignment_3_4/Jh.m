function [ J ] = Jh( wall_params, x, u, Ts )
%Jacobian of measurement equation Jh = dh/dt
%   x:      state
%   u:      input
%   Ts:     sample time
%   J:      Jacobian

a1 = wall_params(1,1);
b1 = wall_params(1,2);
c1 = wall_params(1,3);
a2 = wall_params(2,1);
b2 = wall_params(2,2);
c2 = wall_params(2,3);

J = [-a1/sqrt(a1^2+b1^2),   -b1/sqrt(a1^2+b1^2), 0;
     -a2/sqrt(a2^2+b2^2),   -b2/sqrt(a2^2+b2^2), 0];
end

