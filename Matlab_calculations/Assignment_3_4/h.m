function [ y ] = h( wall_params, x, u, Ts )
% Measurement equation of system y(k) = h(x(k), u(k), Ts)
%   y:      state at index k
%   u:      input at index k
%   Ts:     sample time
%   y:      measurement at index k

a1 = wall_params(1,1);
b1 = wall_params(1,2);
c1 = wall_params(1,3);
a2 = wall_params(2,1);
b2 = wall_params(2,2);
c2 = wall_params(2,3);

alpha = 0.0875;
beta = 0.065;
gamma = 0.0855;

y =  [((a1*(x(1)+alpha))+(b1*x(2))-c1)/sqrt((a1^2)+(b1^2));
      ((a2*(x(1)-beta))+(b2*(x(2)+gamma))-c2)/sqrt((a2^2)+(b2^2))];
end

