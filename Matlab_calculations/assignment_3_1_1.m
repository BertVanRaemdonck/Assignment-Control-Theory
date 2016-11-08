close all
clear all
clc
clear

%% Parameters
fs = 100;   % Hz
Ts = 1/fs;  % s

%% Viewing data
rec_random1 = readlog('log_gpio_random1.xml');

t = rec_random1.getData('time');
v = rec_random1.getData('Voltage');
enc1 = rec_random1.getData('Encoder1');
enc2 = rec_random1.getData('Encoder2');

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


%% Unwrapping the values of the encoder

enc_bits = 16;  % amount of bits used by the encoder
enc1 = cust_unwrap(enc1, enc_bits);
enc2 = cust_unwrap(enc2, enc_bits);

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

% Interpolate the speeds to uniform timesteps since fft assumes the data
% are evenly spaced in time
t_uniform = (t(1):10:t(1)+(length(t)-1)*10)';           % the equivalent of t if Ts were truly uniform
enc1_speed_uniform = interp1(t,enc1_speed,t_uniform);   % values of enc1_speed at the corresponding times
enc2_speed_uniform = interp1(t,enc2_speed,t_uniform);   % values of enc2_speed at the corresponding times

n = size(t,1);
f = fs*(-n/2:n/2-1)/n;
v_f = fftshift(fft(v))/n;
enc1_f = fftshift(fft(enc1_speed_uniform))/n;
enc2_f = fftshift(fft(enc2_speed))/n;

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
        