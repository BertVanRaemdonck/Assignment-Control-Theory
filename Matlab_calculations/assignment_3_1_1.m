if ~ exist('show_figures', 'var')
    close all
    clear all
    clc
    clear
    show_figures = 1;   % show_figures can be set to 0 in another file to suppress the figures in this code
end


%% Parameters
fs = 100;   % Hz
Ts = 1/fs;  % s

type = 'block';         % type of the signal: 'block', 'rand', ...
period = 1000;          % period of the signal, 0 if it isn't periodical
number_of_steps = 1;    % number of iterations for iterative least squares method
butter_cutoff = 0.3;    % between 0 and 1, for butter function, set to 1 to disable filter

type2 = 'block';        % type of the signal: 'block', 'rand', ...
period2 = 2000;         % period of the signal, 0 if it isn't periodical
number_of_steps2 = 1;   % number of iterations for iterative least squares method
butter_cutoff2 = 0.3;   % between 0 and 1, for butter function, set to 1 to disable filter


%% Cleaning and viewing data

% compile file name and import data
if period > 0
    log_name = sprintf('log_gpio_%s_(%d).xml', type, period);
else
    log_name = sprintf('log_gpio_%s.xml', type);
end
rec = readlog(log_name);

period = period + 1;

% raw input data - these are sampled at a non-uniform rate!
t_input = rec.getData('time');
v_input = rec.getData('Voltage');
enc1_input = rec.getData('Encoder1');
enc2_input = rec.getData('Encoder2');

% Unwrapping the values of the encoder
enc_bits = 16;  % amount of bits used by the encoder
enc1_input = cust_unwrap(enc1_input, enc_bits);
enc2_input = cust_unwrap(enc2_input, enc_bits);

% Interpolate the input data to uniform timesteps 
t = (t_input(1):Ts*1e3:t_input(1)+(length(t_input)-1)*Ts*1e3)'; % the equivalent of t_input if Ts were truly uniform
v = v_input;                                                    % v doesn't have to be interpolated because the Arduino does a zoh                  
enc1 = interp1(t_input,enc1_input,t);                           % values of enc1 at the corresponding times
enc2 = interp1(t_input,enc2_input,t);                           % values of enc2 at the corresponding times
t = 1e-3*(t - t_input(1));                                      % set t to be in s and start at 0 for convenience

% Apply average filter
v = average_filter(v,period);
enc1 = average_filter(enc1,period);
enc2 = average_filter(enc2,period);
t = t(1:length(v));

% Plot the data
if show_figures == 1
    figure('name', 'Raw input data')
    subplot(3,1,1)
    plot(t, v);
    xlabel('t [ms]')
    ylabel('v [mV]')
    subplot(3,1,2)
    plot(t, enc1);
    xlabel('t [ms]')
    ylabel('\theta_1 [enc]')
    subplot(3,1,3)
    plot(t, enc2);
    xlabel('t [ms]')
    ylabel('\theta_2 [enc]')
end


%% Calculating speeds

enc1_v = central_diff(enc1, t);
enc2_v = central_diff(enc2, t);

% Extra butterworth filter against noise
if (0 < butter_cutoff) && (butter_cutoff < 1)
    [B_filt, A_filt] = butter(6, butter_cutoff);
    v = filter(B_filt, A_filt, v);
    enc1_v = filter(B_filt, A_filt, enc1_v-enc1_v(1)) + enc1_v(1);
    enc2_v = filter(B_filt, A_filt, enc2_v-enc2_v(1)) + enc2_v(1);
end

if show_figures == 1
    figure('name', 'Processed input data')
    subplot(3,1,1)
    plot(t, v);
    xlabel('t [ms]')
    ylabel('v [mV]')
    subplot(3,1,2)
    plot(t, enc1_v);             
    xlabel('t [ms]')
    ylabel('\omega_1 [enc/s]')
    subplot(3,1,3)
    plot(t, enc2_v);             
    xlabel('t [ms]')
    ylabel('\omega_2 [enc/s]')
end

%% Converting to frequency domain

nb_freqs = size(t,1);
f = fs*(-nb_freqs/2:nb_freqs/2-1)/nb_freqs;
v_f = fftshift(fft(v,nb_freqs))/nb_freqs;
enc1_v_f = fftshift(fft(enc1_v,nb_freqs))/nb_freqs;
enc2_v_f = fftshift(fft(enc2_v,nb_freqs))/nb_freqs;

if show_figures == 1
    figure('name', 'Input data in the frequency domain')
    subplot(3,2,1)
    plot(f, abs(v_f))
    xlabel('f [Hz]')
    ylabel('|v|')
    subplot(3,2,2)
    plot(f, 180/pi*unwrap(angle(v_f)))
    xlabel('f [Hz]')
    ylabel('\anglev [°]')

    subplot(3,2,3)
    plot(f, abs(enc1_v_f))
    xlabel('f [Hz]')
    ylabel('|enc1|')
    subplot(3,2,4)
    plot(f, 180/pi*unwrap(angle(enc1_v_f)))
    xlabel('f [Hz]')
    ylabel('\angleenc1 [°]')

    subplot(3,2,5)
    plot(f, abs(enc2_v_f))
    xlabel('f [Hz]')
    ylabel('|enc2|')
    subplot(3,2,6)
    plot(f, 180/pi*unwrap(angle(enc2_v_f)))
    xlabel('f [Hz]')
    ylabel('\angleenc2 [°]')
end


%% Finding least squares solution

[v_filt_enc1, enc1_v_filt, sys_enc1, num_enc1, den_enc1] = ...
        least_squares_filtered(v,enc1_v,number_of_steps, Ts);
[v_filt_enc2, enc2_v_filt, sys_enc2, num_enc2, den_enc2] = ...
        least_squares_filtered(v,enc2_v,number_of_steps, Ts);

% calculation poles and zeros
[omega_n_enc1_cont, zeta_enc1, poles_enc1] = damp(sys_enc1);
zeros_enc1 = zero(sys_enc1)
[omega_n_enc2_cont, zeta_enc2, poles_enc2] = damp(sys_enc2);
zeros_enc2 = zero(sys_enc2)

% plotting bodeplots
if show_figures == 1
    figure('name','Bodeplots calculated systems')
    subplot(2,1,1)
    bode(sys_enc1)
    grid on
    title('Bodeplot encoder 1')
    subplot(2,1,2)
    bode(sys_enc2)
    grid on
    title('Bodeplot encoder2')
end

%% Checking least squares approximation in frequency domain

FRF_enc1 = freqz(num_enc1, den_enc1, f, fs);
FRF_enc2 = freqz(num_enc2, den_enc2, f, fs);

if show_figures == 1
    figure('name','Comparison bode plots')
    subplot(2,2,1)
    semilogx(f, 20*log10(abs(enc1_v_f./v_f)), f, 20*log10(abs(FRF_enc1)), '--', 'LineWidth', 1)
    grid on
    axis tight
    xlabel('f  [Hz]')
    ylabel('|FRF|  [dB]')
    legend('emp', 'est')
    title('encoder 1')
    subplot(2,2,3)
    semilogx(f, 180/pi*unwrap(angle(enc1_v_f./v_f)), f, 180/pi*unwrap(angle(FRF_enc1)), '--', 'LineWidth', 1)
    grid on
    axis tight
    xlabel('f  [Hz]')
    ylabel('\phi(FRF)  [^\circ]')
    legend('emp', 'est')

    subplot(2,2,2)
    semilogx(f, 20*log10(abs(enc2_v_f./v_f)), f, 20*log10(abs(FRF_enc2)), '--', 'LineWidth', 1)
    grid on
    axis tight
    xlabel('f  [Hz]')
    ylabel('|FRF|  [dB]')
    legend('emp', 'est')
    title('encoder 2')
    subplot(2,2,4)
    semilogx(f, 180/pi*unwrap(angle(enc2_v_f./v_f)), f, 180/pi*unwrap(angle(FRF_enc2)), '--', 'LineWidth', 1)
    grid on
    axis tight
    xlabel('f  [Hz]')
    ylabel('\phi(FRF)  [^\circ]')
    legend('emp', 'est')
end

%% Checking least squares approximation in time domain
sys_enc1_ss = ss(sys_enc1); % state space equivalent of the system, needed to set initial conditions for lsim correctly, see https://nl.mathworks.com/matlabcentral/answers/99019-how-can-i-set-the-initial-value-for-the-output-y-1-when-using-the-lsim-function-in-control-syst
sys_enc2_ss = ss(sys_enc2);

enc1_v_est = lsim(sys_enc1_ss, v, t, [0;enc1_v(1)/sys_enc1_ss.C(end)], '--');
enc2_v_est = lsim(sys_enc2_ss, v, t, [0;enc2_v(1)/sys_enc2_ss.C(end)], '--');

if show_figures == 1
    figure('name','Comparison time response')
    subplot(2,1,1)
    plot(t,enc1_v)
    hold on
    plot(t,enc1_v_est,'--')
    xlabel('t [s]')
    ylabel('\omega_1 [enc/s]')
    legend('emp','est','location','northwest')
    title('encoder 1')
    hold off

    subplot(2,1,2)
    plot(t,enc2_v)
    hold on
    plot(t,enc2_v_est,'--')
    xlabel('t [s]')
    ylabel('\omega_2 [enc/s]')
    legend('emp','est','location','northwest')
    title('encoder 2')
    hold off
end
    

%% Checking model for other input
% Cleaning and viewing data

if period2 > 0
    log_name2 = sprintf('log_gpio_%s_(%d).xml', type2, period2);
else
    log_name2 = sprintf('log_gpio_%s.xml', type2);
end
rec_random2 = readlog(log_name2);

period2 = period2 + 1;

% raw input data - these are sampled at a non-uniform rate!
t_input2 = rec_random2.getData('time');
v_input2 = rec_random2.getData('Voltage');
enc1_input2 = rec_random2.getData('Encoder1');
enc2_input2 = rec_random2.getData('Encoder2');

% Unwrapping the values of the encoder
enc_bits = 16;  % amount of bits used by the encoder
enc1_input2 = cust_unwrap(enc1_input2, enc_bits);
enc2_input2 = cust_unwrap(enc2_input2, enc_bits);

% Interpolate the input data to uniform timesteps 
t2 = (t_input2(1):Ts*1e3:t_input2(1)+(length(t_input2)-1)*Ts*1e3)'; % the equivalent of t_input if Ts were truly uniform
v2 = v_input2;                                                      % v doesn't have to be interpolated because the Arduino does a zoh                  
enc12 = interp1(t_input2,enc1_input2,t2);                           % values of enc1 at the corresponding times
enc22 = interp1(t_input2,enc2_input2,t2);                           % values of enc2 at the corresponding times
t2 = 1e-3*(t2 - t_input2(1));

% Apply average filter
v2 = average_filter(v2,period2);
enc12 = average_filter(enc12,period2);
enc22 = average_filter(enc22,period2);
t2 = t2(1:length(v2));

% Calculating speeds
enc12_speed = central_diff(enc12, t2);
enc12_speed1 = enc12_speed(1);                 % must be saved for later
enc12_speed = enc12_speed - enc12_speed(1);     % must start with zero value for fft
enc22_speed = central_diff(enc22, t2);
enc22_speed1 = enc22_speed(1);                 % must be saved for later
enc22_speed = enc22_speed - enc22_speed(1);     % must start with zero value for fft


% Extra butterworth filter against noise  DIDN'T WORK ??
[B_filt2, A_filt2] = butter(6,butter_cutoff2);
v2 = filter(B_filt2, A_filt2, v2);
enc12_speed = filter(B_filt2, A_filt2, enc12_speed);
enc22_speed = filter(B_filt2, A_filt2, enc22_speed);

% encoder 1
enc12_speed = enc12_speed + enc12_speed1;          % compensate back for setting speed(1) to zero
enc22_speed = enc22_speed + enc22_speed1;          % compensate back for setting speed(1) to zero

% Time simulation of system encoder 1 for input 2
time2=0:Ts:(length(v2)-1)*Ts;
y2_enc1 = lsim(sys_enc1,v2,time2,'--');    

if show_figures == 1
    figure('name','lsim encoder 1 input 2')
    plot(time2,enc12_speed)
    hold on
    plot(time2,y2_enc1,'--')
    xlabel('t [s]')
    ylabel('\omega_1 [enc/s]')
    legend('emp','est')
    hold off
end

% Time simulation of system encoder 2 input 2
y2_enc2 = lsim(sys_enc2,v2,time2,'--');     

if show_figures == 1
    figure('name','lsim encoder 2 input 2')
    plot(time2,enc22_speed)
    hold on
    plot(time2,y2_enc2,'--')
    xlabel('t [s]')
    ylabel('\omega_2 [enc/s]')
    legend('emp','est')
    hold off
end


clear show_figures

