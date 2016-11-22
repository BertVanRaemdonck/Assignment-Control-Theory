close all
clear all
clc
clear

%% Assignment_3_1_1.m opvragen
show_figures = 0;   % assignment_3_1_1 doesn't display its figures so works a bit faster
assignment_3_1_1


%% Parameters, getting data and calculations

close all
clc

% parameters
low_exp_w = -1;     % w = logspace(low_exp_w,high_exp_w,number_points_w)
high_exp_w = 3;
number_points_w = 1000;

nb_speed_ref = 300;    % number of points of vector with the reference speed it has to track
mag_speed_ref = 4000;   % max magnitude of vector with the reference speed it has to track
speed_ref = (mag_speed_ref/4)*ones([1,nb_speed_ref/3]);
speed_ref = [speed_ref  (mag_speed_ref/2)*ones([1,nb_speed_ref/3])];
speed_ref = [speed_ref  (mag_speed_ref/1)*ones([1,nb_speed_ref/3])];

PM1 = 55;           % desired phase marge of controller 1
omega_co1 = 30;     % desired crossover frequency in rad/s
PM2 = 55;           % desired phase marge of controller 2
omega_co2 = 30;     % desired crossover frequency in rad/s

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
title('Bode diagram for encoder 1')
subplot(2,1,2)
bode(sys_enc2_or, sys_enc2_c),grid
legend('enc2 disc', 'enc2 cont')
title('Bode diagram for encoder 2')

% calculating magnitude and phase for needed calculations
w = logspace(low_exp_w,high_exp_w,number_points_w);

[mag1_c,ph1_c] = bode(sys_enc1_c.num, sys_enc1_c.den, w);
[mag2_c,ph2_c] = bode(sys_enc2_c.num, sys_enc2_c.den, w);



%% Design procedure PI encoder

phase_PI1 = -(-180 + PM1 - interp1(w, ph1_c, omega_co1));   % phase of the PI controller at the crossover frequency for encoder 1
phase_PI2 = -(-180 + PM2 - interp1(w, ph2_c, omega_co2));   % phase of the PI controller at the crossover frequency for encoder 2

T_i1 = tan(pi/180*(90 - phase_PI1))/omega_co1;          % time constant of PI encoder 1
T_i2 = tan(pi/180*(90 - phase_PI2))/omega_co2;          % time constant of PI encoder 2
mag_PI_co1 = abs(1/(j*omega_co1)*(j*omega_co1+1/T_i1)); % magnitude of PI controller of encoder 1 at crossover frequency
mag_PI_co2 = abs(1/(j*omega_co2)*(j*omega_co2+1/T_i2)); % magnitude of PI controller of encoder 2 at crossover frequency
K_1 = 1/(mag_PI_co1 * interp1(w, mag1_c, omega_co1));   % proportional constant of PI controller encoder 1
K_2 = 1/(mag_PI_co2 * interp1(w, mag2_c, omega_co2));   % proportional constant of PI controller encoder 2

num_PI1 = [K_1*T_i1 K_1];        % D(s) = (K * T_i * s + K)/(T_i * s)
num_PI2 = [K_2*T_i2 K_2];
den_PI1 = [T_i1 0];
den_PI2 = [T_i2 0];
sys_PI1 = tf(num_PI1, den_PI1);   
sys_PI2 = tf(num_PI2, den_PI2);  

figure('name', 'Bodeplot of the PI compensators')
subplot(2,1,1)
bode(sys_PI1, w), grid
title('Bode diagram PI for encoder 1')
subplot(2,1,2)
bode(sys_PI1, w), grid
title('Bode diagram PI for encoder 2')

%% Viewing compensated systems

sys_enc1_PIcomp_c = series(sys_PI1, sys_enc1_c);
sys_enc1_PIcomp_d = c2d(sys_enc1_PIcomp_c,Ts);

sys_enc2_PIcomp_c = series(sys_PI2, sys_enc2_c);
sys_enc2_PIcomp_d = c2d(sys_enc2_PIcomp_c,Ts);

figure('name', 'Bodeplot of the compensated systems in continuous time')
subplot(2,1,1)
bode(sys_enc1_c, sys_enc1_PIcomp_c, w), grid
title('Bode diagram for the compensated encoder 1')
legend('enc1', 'enc1_PIcomp')
subplot(2,1,2)
bode(sys_enc2_c, sys_enc2_PIcomp_c, w), grid
title('Bode diagram for the compensated encoder 2')
legend('enc2', 'enc2_PIcomp')

figure('name', 'Bodeplot of the compensated systems in discrete time')
subplot(2,1,1)
bode(sys_enc1_or, sys_enc1_PIcomp_d, w), grid
legend('enc1', 'enc1_PIcomp')
title('Bode diagram for the compensated encoder 1')
subplot(2,1,2)
bode(sys_enc2_or, sys_enc2_PIcomp_d, w), grid
legend('enc2', 'enc2_PIcomp')
title('Bode diagram for the compensated encoder 2')


%% Testing controller

% Making feedback loop
sys_enc1_or_fb = feedback(sys_enc1_or,1);
sys_enc2_or_fb = feedback(sys_enc2_or,1);

sys_enc1_PIcomp_fb = feedback(sys_enc1_PIcomp_d,1);
sys_enc2_PIcomp_fb = feedback(sys_enc2_PIcomp_d,1);

% Time simulation of system encoders with controller
time_fb =0:Ts:(length(speed_ref)-1)*Ts;
y1_enc1_or_fb = lsim(sys_enc1_or_fb,speed_ref,time_fb);
y1_enc1_PIcomp_fb = lsim(sys_enc1_PIcomp_fb,speed_ref,time_fb,'--'); 

y1_enc2_or_fb = lsim(sys_enc2_or_fb,speed_ref,time_fb);
y1_enc2_PIcomp_fb = lsim(sys_enc2_PIcomp_fb,speed_ref,time_fb,'--');    

figure('name','lsim encoders with feedback loop')
subplot(2,1,1)
plot(time_fb,speed_ref)
hold on
plot(time_fb,y1_enc1_or_fb,':')
hold on
plot(time_fb,y1_enc1_PIcomp_fb,'--')
xlabel('t [s]')
ylabel('speed [?]')
legend('reference input','without controller','with controller')
title('encoder 1')
hold off
subplot(2,1,2)
plot(time_fb,speed_ref)
hold on
plot(time_fb,y1_enc2_or_fb,':')
hold on
plot(time_fb,y1_enc2_PIcomp_fb,'--')
xlabel('t [s]')
ylabel('speed [?]')
legend('reference input','without controller','with controller')
title('encoder 2')
hold off

%% Data needed to implement controller

sys_PI1_d = c2d(sys_PI1, Ts, 'zoh');
sys_PI2_d = c2d(sys_PI2, Ts, 'zoh');
