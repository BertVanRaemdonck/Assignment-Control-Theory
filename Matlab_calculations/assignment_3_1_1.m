close all
clear all
clc
clear

%% Parameters
fs = 100;   % Hz
Ts = 1/fs;  % s

type = 'block'           % type of the signal: 'block', 'rand', ...
period = 1000;          % period of the signal, 0 if it isn't periodical
number_of_steps = 1;    % number of iterations for iterative least squares method
factor_butter = 0.3;    % between 0 and 1, for butter function

type2 = 'block'           % type of the signal: 'block', 'rand', ...
period2 = 2000;          % period of the signal, 0 if it isn't periodical
number_of_steps2 = 1;    % number of iterations for iterative least squares method
factor_butter2 = 0.3;    % between 0 and 1, for butter function

schaal_lsim_input = 1%*1e-3;     % scale used for scaling lsim, should be 1, but doesn't work :(
schaal_lsim_output = 1%*2.2e3;

%% Cleaning and viewing data

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
enc1_speed1 = enc1_speed(1);                 % must be saved for later
enc1_speed = enc1_speed - enc1_speed(1);     % must start with zero value for fft
enc2_speed = central_diff(enc2, t);
enc2_speed1 = enc2_speed(1);                 % must be saved for later
enc2_speed = enc2_speed - enc2_speed(1);     % must start with zero value for fft


% Extra butterworth filter against noise  DIDN'T WORK ??
[B_filt, A_filt] = butter(6,factor_butter);
v = filter(B_filt, A_filt, v);
enc1_speed = filter(B_filt, A_filt, enc1_speed);
enc2_speed = filter(B_filt, A_filt, enc2_speed);

figure('name', 'Processed input data')
subplot(3,1,1)
plot(t, v);
xlabel('t [ms]')
ylabel('v [mV]')
subplot(3,1,2)
plot(t, enc1_speed + enc1_speed1);              % compensate back for setting speed(1) to zero
xlabel('t [ms]')
ylabel('speed encoder 1')
subplot(3,1,3)
plot(t, enc2_speed + enc2_speed1);              % compensate back for setting speed(1) to zero
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
ylabel('\anglev [�]')

subplot(3,2,3)
plot(f, abs(enc1_f))
xlabel('f [Hz]')
ylabel('|enc1|')
subplot(3,2,4)
plot(f, unwrap(angle(enc1_f)))
xlabel('f [Hz]')
ylabel('\angleenc1 [�]')

subplot(3,2,5)
plot(f, abs(enc2_f))
xlabel('f [Hz]')
ylabel('|enc2|')
subplot(3,2,6)
plot(f, unwrap(angle(enc2_f)))
xlabel('f [Hz]')
ylabel('\angleenc2 [�]')


figure('name', 'Bodeplot overdrachtsfunctie v -> enc1_speed')
subplot(2,1,1)
semilogx(f, 20*log10(abs(enc1_f./v_f)))
xlabel('f [Hz]')
ylabel('|H_{v,enc1}| [dB]')
subplot(2,1,2)
semilogx(f, 180/pi*unwrap(angle(enc1_f./v_f)))
xlabel('f [Hz]')
ylabel('\angleH_{v,enc1} [�]')


figure('name', 'Bodeplot overdrachtsfunctie v -> enc2_speed')
subplot(2,1,1)
semilogx(f, 20*log10(abs(enc2_f./v_f)))
xlabel('f [Hz]')
ylabel('|H_{v,enc2}| [dB]')
subplot(2,1,2)
semilogx(f, 180/pi*unwrap(angle(enc2_f./v_f)))
xlabel('f [Hz]')
ylabel('\angleH_{v,enc2} [�]')

%  % If you want te see all bode plots together, uncomment this section
% figure('name', 'Bodeplot overdrachtsfunctie vergelijken')
% subplot(2,2,1)
% semilogx(f, 20*log10(abs(enc1_f./v_f)))
% xlabel('f [Hz]')
% ylabel('|H_{v,enc1}| [dB]')
% subplot(2,2,3)
% semilogx(f, 180/pi*unwrap(angle(enc1_f./v_f)))
% xlabel('f [Hz]')
% ylabel('\angleH_{v,enc1} [�]')
% subplot(2,2,2)
% semilogx(f, 20*log10(abs(enc2_f./v_f)))
% xlabel('f [Hz]')
% ylabel('|H_{v,enc2}| [dB]')
% subplot(2,2,4)
% semilogx(f, 180/pi*unwrap(angle(enc2_f./v_f)))
% xlabel('f [Hz]')
% ylabel('\angleH_{v,enc2} [�]')


%% Finding least squares solution
% encoder 1
enc1_speed = enc1_speed + enc1_speed1;          % compensate back for setting speed(1) to zero
enc2_speed = enc2_speed + enc2_speed1;          % compensate back for setting speed(1) to zero

[v_filt1,enc1_speed_filt,sys1, teller1, noemer1] = least_squares_filtered(v,enc1_speed,number_of_steps, Ts);

%calculation poles and zeros encoder 1
[Wn1, zeta1, P1] = damp(sys1);
omega_n1_cont = Wn1
poles_1 = P1
zeros_1 = zero(sys1)

%Bode plot encoder 1
figure('name','Bodeplot least squares solution encoder 1')
bode(sys1)

FRF_lin1 = freqz(teller1, noemer1, f, fs);
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

% Time simulation of system encoder 1
time=0:Ts:(length(v)-1)*Ts;
y_enc1 = lsim(sys1*schaal_lsim_output,v*(schaal_lsim_input),time,'--');     % FACTOR TOEGEVOEGD!!!!
figure('name','lsim encoder 1')
plot(time,enc1_speed)
hold on
plot(time,y_enc1,'--')
xlabel('t [s]')
ylabel('speed [?]')
legend('emp','est')
hold off


% encoder 2
[v_filt2,enc2_speed_filt,sys2, teller2, noemer2] = least_squares_filtered(v,enc2_speed,number_of_steps, Ts);

%calculation poles and zeros encoder 1
[Wn2, zeta2, P2] = damp(sys2);
omega_n2_cont = Wn2
poles_2 = P2
zeros_2 = zero(sys2)

%Bode plot encoder 1
figure('name','Bodeplot least squares solution encoder 2')
bode(sys2)

FRF_lin2 = freqz(teller2, noemer2, f, fs);
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

% Time simulation of system encoder 2
y_enc2 = lsim(sys2*schaal_lsim_output,v*(schaal_lsim_input),time,'--');    % FACTOR TOEGEVOEGD!!!!

figure('name','lsim encoder 2')
plot(time,enc2_speed)
hold on
plot(time,y_enc2,'--')
xlabel('t [s]')
ylabel('speed [?]')
legend('emp','est')
hold off


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
v2 = v_input2;                                                    % v doesn't have to be interpolated because the Arduino does a zoh                  
enc12 = interp1(t_input2,enc1_input2,t);                           % values of enc1 at the corresponding times
enc22 = interp1(t_input2,enc2_input2,t);                           % values of enc2 at the corresponding times


% Apply average filter
v2 = average_filter(v2,period);
enc12 = average_filter(enc12,period);
enc22 = average_filter(enc22,period);
t2 = t2(1:length(v2));

% Calculating speeds
enc12_speed = central_diff(enc12, t);
enc12_speed1 = enc12_speed(1);                 % must be saved for later
enc12_speed = enc12_speed - enc12_speed(1);     % must start with zero value for fft
enc22_speed = central_diff(enc22, t);
enc22_speed1 = enc22_speed(1);                 % must be saved for later
enc22_speed = enc22_speed - enc22_speed(1);     % must start with zero value for fft


% Extra butterworth filter against noise  DIDN'T WORK ??
[B_filt2, A_filt2] = butter(6,factor_butter2);
v2 = filter(B_filt2, A_filt2, v2);
enc12_speed = filter(B_filt2, A_filt2, enc12_speed);
enc22_speed = filter(B_filt2, A_filt2, enc22_speed);

% encoder 1
enc12_speed = enc12_speed + enc12_speed1;          % compensate back for setting speed(1) to zero
enc22_speed = enc22_speed + enc22_speed1;          % compensate back for setting speed(1) to zero

% Time simulation of system encoder 1 for input 2
time2=0:Ts:(length(v2)-1)*Ts;
y2_enc1 = lsim(sys1*schaal_lsim_output,v2*(schaal_lsim_input),time2,'--');     % FACTOR TOEGEVOEGD!!!!

figure('name','lsim encoder 1 input 2')
plot(time2,enc12_speed)
hold on
plot(time2,y2_enc1,'--')
xlabel('t [s]')
ylabel('speed [?]')
legend('emp','est')
hold off

% Time simulation of system encoder 2 input 2
y2_enc2 = lsim(sys2*schaal_lsim_output,v2*(schaal_lsim_input),time2,'--');     % FACTOR TOEGEVOEGD!!!!

figure('name','lsim encoder 2 input 2')
plot(time2,enc22_speed)
hold on
plot(time2,y2_enc2,'--')
xlabel('t [s]')
ylabel('speed [?]')
legend('emp','est')
hold off




