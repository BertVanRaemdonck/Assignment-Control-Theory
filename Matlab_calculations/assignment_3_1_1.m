close all
clear all
clc
clear

%% Parameters
fs = 100;   % Hz
Ts = 1/fs;  % s

%% Cleaning and viewing data
rec_random1 = readlog('log_gpio_rand_(1000).xml');
period =1000;

% raw input data - these are sampled at a non-uniform rate!
t_input = rec_random1.getData('time');
v_input = rec_random1.getData('Voltage');
enc1_input = rec_random1.getData('Encoder1');
enc2_input = rec_random1.getData('Encoder2');

% Unwrapping the values of the encoder
enc_bits = 16;  % amount of bits used by the encoder
enc1_input = cust_unwrap(enc1_input, enc_bits);
enc2_input = cust_unwrap(enc2_input, enc_bits);

% Interpolate the input data to uniform timesteps 
t = (t_input(1):Ts*1e3:t_input(1)+(length(t_input)-1)*Ts*1e3)'; % the equivalent of t_input if Ts were truly uniform
v = interp1(t_input,v_input,t);                                 % values of v at the corresponding times
enc1 = interp1(t_input,enc1_input,t);                           % values of enc1 at the corresponding times
enc2 = interp1(t_input,enc2_input,t);                           % values of enc2 at the corresponding times

% Apply average filter
v = average_filter(v,period);
enc1 = average_filter(enc2,period);
enc2 = average_filter(enc2,period);
t = t(1:length(v));

% Plot the data
figure('name', 'Raw input data')
subplot(3,1,1)
plot(t, v);
xlabel('t [ms]')
ylabel('v [mV]')
subplot(3,1,2)
plot(t, enc1);
xlabel('t [ms]')
ylabel('value encoder 1')
subplot(3,1,3)
plot(t, enc2);
xlabel('t [ms]')
ylabel('value encoder 2')




%% Calculating speeds

enc1_speed = central_diff(enc1, t);
enc2_speed = central_diff(enc2, t);

figure('name', 'Processed input data')
subplot(3,1,1)
plot(t, v);
xlabel('t [ms]')
ylabel('v [mV]')
subplot(3,1,2)
plot(t, enc1_speed);
xlabel('t [ms]')
ylabel('speed encoder 1')
subplot(3,1,3)
plot(t, enc2_speed);
xlabel('t [ms]')
ylabel('speed encoder 2')

%% Converting to frequency domain

% % Interpolate the speeds to uniform timesteps since fft assumes the data
% % are evenly spaced in time
% t_uniform = (t(1):10:t(1)+(length(t)-1)*10)';           % the equivalent of t if Ts were truly uniform
% enc1_speed_uniform = interp1(t,enc1_speed,t_uniform);   % values of enc1_speed at the corresponding times
% enc2_speed_uniform = interp1(t,enc2_speed,t_uniform);   % values of enc2_speed at the corresponding times

n = size(t,1);
f = fs*(-n/2:n/2-1)/n;
v_f = fftshift(fft(v,n))/n;
enc1_f = fftshift(fft(enc1_speed,n))/n;
enc2_f = fftshift(fft(enc2_speed,n))/n;

figure('name', 'Input data in the frequency domain')
subplot(3,2,1)
plot(f, abs(v_f))
xlabel('f [Hz]')
ylabel('|v|')
subplot(3,2,2)
plot(f, unwrap(angle(v_f)))
xlabel('f [Hz]')
ylabel('\anglev [°]')

subplot(3,2,3)
plot(f, abs(enc1_f))
xlabel('f [Hz]')
ylabel('|enc1|')
subplot(3,2,4)
plot(f, unwrap(angle(enc1_f)))
xlabel('f [Hz]')
ylabel('\angleenc1 [°]')

subplot(3,2,5)
plot(f, abs(enc2_f))
xlabel('f [Hz]')
ylabel('|enc2|')
subplot(3,2,6)
plot(f, unwrap(angle(enc2_f)))
xlabel('f [Hz]')
ylabel('\angleenc2 [°]')


figure('name', 'Bodeplot overdrachtsfunctie v -> enc1_speed')
subplot(2,1,1)
semilogx(f, 20*log10(abs(enc1_f./v_f)))
xlabel('f [Hz]')
ylabel('|H_{v,enc1}| [dB]')
subplot(2,1,2)
semilogx(f, 180/pi*unwrap(angle(enc1_f./v_f)))
xlabel('f [Hz]')
ylabel('\angleH_{v,enc1} [°]')
