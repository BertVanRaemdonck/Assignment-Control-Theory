close all
clear all
clc
clear

%% Assignment_3_1_1.m opvragen
assignment_3_1_1



%% Parameters, getting data and calculations

close all
clc

% parameters
delta_phi = 45;  %graden
add_phase_lag = 15; %graden
low_exp_w = -1;     % w = logspace(low_exp_w,high_exp_w,number_points_w)
high_exp_w = 3;
number_points_w = 1000;

PM_des = 55;        % desired phase marge
phase_PI = 40;      % phase that PI controller may 'eat'

% Taking data from assignment_3_1_1.m
num_enc1_or = num_enc1;
den_enc1_or = den_enc1;
num_enc2_or = num_enc2;
den_enc2_or = den_enc2;

sys_enc1_or = tf(num_enc1_or,den_enc1_or,Ts);
sys_enc2_or = tf(num_enc2_or,den_enc2_or,Ts);

sys_enc1_c = d2c(sys_enc1_or);    % We have to work in continuous time
sys_enc2_c = d2c(sys_enc2_or);

figure('name','Bodeplot of encoders without PI in discrete and continuous time')
subplot(2,1,1)
bode(sys_enc1_or, sys_enc1_c),grid
legend('enc1 disc', 'enc1 cont')
subplot(2,1,2)
bode(sys_enc2_or, sys_enc2_c),grid
legend('enc2 disc', 'enc2 cont')

% calculating magnitude and phase for needed calculations
w = logspace(low_exp_w,high_exp_w,number_points_w);

[mag1_c,ph1_c] = bode(sys_enc1_c.num, sys_enc1_c.den, w);
[mag2_c,ph2_c] = bode(sys_enc2_c.num, sys_enc2_c.den, w);



%% design procedure PI encoder 1

phase_co1 = -180 + PM_des + phase_PI;       % phase we want
omega_co1 = interp1(ph1_c, w, phase_co1);  % frequency with phase we want, will become new crossover frequency

T_i1 = tan(pi/180*(90 - phase_PI))/omega_co1;  % integrated time constant of PI encoder 1
mag_PI_co1 = abs(1/(j*omega_co1)*(j*omega_co1+1/T_i1));  % magnitude of PI controller at crossover frequency
K_1 = 1/(mag_PI_co1 * interp1(w, mag1_c, omega_co1));   % proportional constant of PI controller encoder 1

num_PI1 = [K_1*T_i1 K_1];        % D(s) = (K * T_i * s + K)/(T_i * s)
den_PI1 = [T_i1 0];
sys_PI1 = tf(num_PI1, den_PI1);    

figure('name', 'Bodeplot of the PI compensator encoder 1')
bode(sys_PI1, w), grid

sys_enc1_PIcomp_c = series(sys_PI1, sys_enc1_c);
sys_enc1_PIcomp_d = c2d(sys_enc1_PIcomp_c,Ts)

figure('name', 'Bodeplot of the compensated system of encoder 1 continuous time')
bode(sys_enc1_c, sys_enc1_PIcomp_c, w), grid
legend('enc1', 'enc1_PIcomp')

figure('name', 'Bodeplot of the compensated system of encoder 1 discrete time')
bode(sys_enc1_or, sys_enc1_PIcomp_d, w), grid
legend('enc1', 'enc1_PIcomp')



%% design procedure PI encoder 2

phase_co2 = -180 + PM_des + phase_PI;       % phase we want
omega_co2 = interp1(ph2_c, w, phase_co2);  % frequency with phase we want, will become new crossover frequency

T_i2 = tan(pi/180*(90 - phase_PI))/omega_co2;  % integrated time constant of PI encoder 2
mag_PI_co2 = abs(1/(j*omega_co2)*(j*omega_co2+1/T_i2));  % magnitude of PI controller at crossover frequency
K_2 = 1/(mag_PI_co2 * interp1(w, mag2_c, omega_co2));   % proportional constant of PI controller encoder 2

num_PI2 = [K_2*T_i2 K_2];        % D(s) = (K * T_i * s + K)/(T_i * s)
den_PI2 = [T_i2 0];
sys_PI2 = tf(num_PI2, den_PI2);    

figure('name', 'Bodeplot of the PI compensator encoder 2')
bode(sys_PI2, w), grid

sys_enc2_PIcomp_c = series(sys_PI2, sys_enc2_c);
sys_enc2_PIcomp_d = c2d(sys_enc2_PIcomp_c,Ts)

figure('name', 'Bodeplot of the compensated system of encoder 2 continuous time')
bode(sys_enc2_c, sys_enc2_PIcomp_c, w), grid
legend('enc2', 'enc2_PIcomp')

figure('name', 'Bodeplot of the compensated system of encoder 2 discrete time')
bode(sys_enc2_or, sys_enc2_PIcomp_d, w), grid
legend('enc2', 'enc2_PIcomp')

