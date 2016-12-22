function [ err_loc ] = local_error( x_ref, x_est )
% Computes state error transformed to local frame X'Y'
%   x_ref:      reference state
%   x_est:      estimated state
%   err_loc:    error in local frame X'Y'

err_glo_x = x_ref(1)-x_est(1);
err_glo_y = x_ref(2)-x_est(2);

Rot = [cos(-x_est(3))     -sin(-x_est(3));
       sin(-x_est(3))     cos(-x_est(3))]; 
   
err_loc_xy = Rot*[err_glo_x;
                  err_glo_y];

err_loc = [err_loc_xy;
            x_ref(3)-x_est(3)]; 
end

     