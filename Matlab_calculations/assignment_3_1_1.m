close all
clear all
clc
clear

%% Parameters
fs = 100;   % Hz
Ts = 1/fs;  % s

%% Viewing data
rec_random1 = readlog('log_gpio_ramp1.xml');

t = rec_random1.getData('time');
v = rec_random1.getData('Voltage');
enc1 = rec_random1.getData('Encoder1');
enc2 = rec_random1.getData('Encoder2');

figure()
subplot(3,1,1)
plot(t, v);
subplot(3,1,2)
plot(t, enc1);
subplot(3,1,3)
plot(t, enc2);


%% Unwrapping the values of the encoder

enc_bits = 16;  % amount of bits used by the encoder
enc1 = cust_unwrap(enc1, enc_bits);
enc2 = cust_unwrap(enc2, enc_bits);

%% Calculating speeds

enc1_speed = central_diff(enc1, t);
enc2_speed = central_diff(enc2, t);

%% Converting to frequency domain
n = size(t,1);
f = fs*(-n/2:n/2-1)/n;
v_f = fftshift(fft(v))/n;
enc1_f = fftshift(fft(enc1))/n;

figure()
subplot(2,1,1)
plot(f, abs(v_f))
subplot(2,1,2)
plot(f, unwrap(angle(v_f)))

figure()
subplot(2,1,1)
semilogx(f, 20*log10(abs(enc1_f./v_f)))
subplot(2,1,2)
semilogx(f, 180/pi*unwrap(angle(enc1_f./v_f)))
        