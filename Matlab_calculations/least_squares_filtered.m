function [data1_filt,data2_filt,sys, teller, noemer] = least_squares_filtered(data1,data2,number_of_steps, Ts)
% LEAST_SQUARES_FILTERED    calculates the transferfunction of the least squares solution
% and the filtered input data
%   [data1_filt,data2_filt,sys, teller, noemer] = least_squares_filtered(data1,data2,number_of_steps, Ts)
%   outputs a sequence of filtered data of the input data, using a
%   iterative least squares method of Sanathanan Koerner procedure. The transferfunction is given with sys, 
%   teller is the nominator, noemer is the denominator of this transferfunction.
%   Ts is the sampling period of the input data.
%   number_of_steps = 1 gives the least squares solution without filtering

data1_gem = data1 - mean(data1);
data2_gem = data2 - mean(data2);
%A = data1_gem(3:end);
% foutief opgesteld eerst output (speed), erna input (voltage)
A = data2_gem(3:end);

%B = [-data1_gem(2:end-1), -data1_gem(1:end-2), data2_gem(2:end-1), data2_gem(1:end-2)];   
 % foutief opgesteld eerst output (speed), erna input (voltage)
B = [-data2_gem(2:end-1), -data2_gem(1:end-2), data1_gem(2:end-1), data1_gem(1:end-2)];
phi= B\A;    %[c, d, a, b]

teller = [0, phi(3), phi(4)];
noemer = [1, phi(1), phi(2)];
sys = tf(teller, noemer, Ts);

if number_of_steps == 1
    data1_filt = data1_gem;
    data2_filt = data2_gem;
end

if number_of_steps > 1
    step = 1;
    while step < number_of_steps
        teller_filt = [0, 0, 1];
        noemer_filt = [1, phi(1), phi(2)];
        
        data1_filt = filter(teller_filt, noemer_filt, data1);
        data2_filt = filter(teller_filt, noemer_filt, data2);
        
        data1_filt_gem = data1_filt - mean(data1_filt);
        data2_filt_gem = data2_filt - mean(data2_filt);
        A = data2_filt_gem(3:end);   % Hier ook aangepast
        B = [-data2_filt_gem(2:end-1), -data2_filt_gem(1:end-2), data1_filt_gem(2:end-1), data1_filt_gem(1:end-2)];  % hier ook aangepast
        phi = B\A;    %[c, d, a, b]

        teller = [0, phi(3), phi(4)];
        noemer = [1, phi(1), phi(2)];
        sys = tf(teller, noemer, Ts);
        
        step = step + 1;
    end
 end


        