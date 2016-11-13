close all
clear all
clc
clear

%% Parameters
fs = 100;   % Hz
Ts = 1/fs;  % s

%% Cleaning and viewing data
type = 'rand'       % type of the signal: 'block', 'rand', ...
period = 1000;      % period of the signal, 0 if it isn't periodical
if period > 0
    log_name = sprintf('log_gpio_%s_(%d).xml', type, period);
else
    log_name = sprintf('log_gpio_%s.xml', type);
end
rec_random1 = readlog(log_name);

period = period + 1;


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
v = v_input;                                                    % v doesn't have to be interpolated because the Arduino does a zoh                  
enc1 = interp1(t_input,enc1_input,t);                           % values of enc1 at the corresponding times
enc2 = interp1(t_input,enc2_input,t);                           % values of enc2 at the corresponding times

% Apply average filter
v = average_filter(v,period);
enc1 = average_filter(enc1,period);
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
enc1_speed = enc1_speed - enc1_speed(1);
enc2_speed = central_diff(enc2, t);
enc2_speed = enc2_speed - enc2_speed(1);

% Extra butterworth filter against noise
[B_filt, A_filt] = butter(6,0.3);
v = filter(B_filt, A_filt, v);
enc1_speed = filter(B_filt, A_filt, enc1_speed);
enc2_speed = filter(B_filt, A_filt, enc2_speed);

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


figure('name', 'Bodeplot overdrachtsfunctie v -> enc2_speed')
subplot(2,1,1)
semilogx(f, 20*log10(abs(enc2_f./v_f)))
xlabel('f [Hz]')
ylabel('|H_{v,enc2}| [dB]')
subplot(2,1,2)
semilogx(f, 180/pi*unwrap(angle(enc2_f./v_f)))
xlabel('f [Hz]')
ylabel('\angleH_{v,enc2} [°]')

%  % If you want te see all bode plots together, uncomment this section
% figure('name', 'Bodeplot overdrachtsfunctie vergelijken')
% subplot(2,2,1)
% semilogx(f, 20*log10(abs(enc1_f./v_f)))
% xlabel('f [Hz]')
% ylabel('|H_{v,enc1}| [dB]')
% subplot(2,2,3)
% semilogx(f, 180/pi*unwrap(angle(enc1_f./v_f)))
% xlabel('f [Hz]')
% ylabel('\angleH_{v,enc1} [°]')
% subplot(2,2,2)
% semilogx(f, 20*log10(abs(enc2_f./v_f)))
% xlabel('f [Hz]')
% ylabel('|H_{v,enc2}| [dB]')
% subplot(2,2,4)
% semilogx(f, 180/pi*unwrap(angle(enc2_f./v_f)))
% xlabel('f [Hz]')
% ylabel('\angleH_{v,enc2} [°]')


%% Finding least squares solution
% encoder 1
u = v - mean(v);
enc1_speed_gem = enc1_speed - mean(enc1_speed);
A = u(3:end);
B1 = [-u(2:end-1), u(1:end-2), enc1_speed_gem(2:end-1), enc1_speed_gem(1:end-2)];
phi_1 = B1\A;    %[c, d, a, b]

teller_1 = [0, phi_1(3), phi_1(4)];
noemer_1 = [1, phi_1(1), phi_1(2)];
sys_d_1 = tf(teller_1, noemer_1, Ts);

%calculation poles and zeros encoder 1
[Wn1, zeta1, P1] = damp(sys_d_1);
omega_n1_cont = Wn1
poles_1 = P1
zeros_1 = zero(sys_d_1)

%Bode plot encoder 1
figure('name','Bodeplot least squares solution encoder 1')
bode(sys_d_1)

FRF_lin1 = freqz(teller_1, noemer_1, f, fs);
figure('name','Comparison bode plots encoder 1')
subplot(2,1,1)
semilogx(f, 20*log10(abs(enc1_f./v_f)), f, 20*log10(abs(FRF_lin1)), '--', 'LineWidth', 1)
grid on
axis tight
xlabel('f  [Hz]')
ylabel('|FRF|  [m]')
legend('emp', 'est')
subplot(2,1,2)
semilogx(f, 180/pi*unwrap(angle(enc1_f./v_f)), f, 180/pi*unwrap(angle(FRF_lin1)), '--', 'LineWidth', 1)
grid on
axis tight
xlabel('f  [Hz]')
ylabel('\phi(FRF)  [^\circ]')
legend('emp', 'est')


% encoder 2
enc2_speed_gem = enc2_speed - mean(enc2_speed);

B2 = [-u(2:end-1), u(1:end-2), enc2_speed_gem(2:end-1), enc2_speed_gem(1:end-2)];
phi_2 = B2\A;    %[c, d, a, b]

teller_2 = [0, phi_2(3), phi_2(4)];
noemer_2 = [1, phi_2(1), phi_2(2)];
sys_d_2 = tf(teller_2, noemer_2, Ts);

%Calculation poles and zeros encoder 2
[Wn2, zeta2, P2] = damp(sys_d_2);
omega_n2_cont = Wn2
poles_2 = P2
zeros_2 = zero(sys_d_2)

%Bode plot encoder 2
figure('name','Bodeplot least squares solution encoder 2')
bode(sys_d_2)

FRF_lin2 = freqz(teller_2, noemer_2, f, fs);
figure('name','Comparison bode plots encoder 2')
subplot(2,1,1)
semilogx(f, 20*log10(abs(enc2_f./v_f)), f, 20*log10(abs(FRF_lin2)), '--', 'LineWidth', 1)
grid on
axis tight
xlabel('f  [Hz]')
ylabel('|FRF|  [m]')
legend('emp', 'est')
subplot(2,1,2)
semilogx(f, 180/pi*unwrap(angle(enc2_f./v_f)), f, 180/pi*unwrap(angle(FRF_lin2)), '--', 'LineWidth', 1)
grid on
axis tight
xlabel('f  [Hz]')
ylabel('\phi(FRF)  [^\circ]')
legend('emp', 'est')


