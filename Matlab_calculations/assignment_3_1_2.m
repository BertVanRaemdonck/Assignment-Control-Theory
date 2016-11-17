close all
clear all
clc
clear

%% Assignment_3_1_1.m opvragen
assignment_3_1_1



%% Code oefenzitting 4

close all
clc

% parameters
delta_phi = 45;  %graden
add_phase_lag = 15; %graden

% Taking data from assignment_3_1_1.m
num_enc1_or = num_enc1;
den_enc1_or = den_enc1;
num_enc2_or = num_enc2;
den_enc2_or = den_enc2;

sys_enc1_or = tf(num_enc1_or,den_enc1_or,Ts);
sys_enc2_or = tf(num_enc2_or,den_enc2_or,Ts);

figure('name','Bodeplot of encoders without PI DISCRETE TIME')
subplot(2,1,1)
bode(sys_enc1_or),grid
subplot(2,1,2)
bode(sys_enc2_or),grid

sys_enc1_or = d2c(sys_enc1_or);    % We have to work in continuous time
sys_enc2_or = d2c(sys_enc2_or);

num_enc1_or = cell2mat(sys_enc1_or.num);
den_enc1_or = cell2mat(sys_enc1_or.den);

probeersel = tf(num_enc1_or,den_enc1_or);


figure('name','Bodeplot of encoders without PI CONTINUOUS TIME')
subplot(2,1,1)
bode(sys_enc1_or),grid
subplot(2,1,2)
bode(sys_enc2_or),grid

% figure('name','probeersel CONTINUOUS TIME')
% bode(sys_enc1_or),grid
% hold on
% bode(probeersel),grid
% hold off

% other method of making bodeplot
w1 = logspace(-1,3,1000);
%[mag1_or,ph1_or,w1] = bode(sys_enc1_or);
[mag1_or,ph1_or] = bode(num_enc1_or,den_enc1_or,w1);
%[mag2_or,ph2_or,w2] = bode(sys_enc2_or);
[mag2_or,ph2_or] = bode(num_enc2_or,den_enc2_or,w1);

mag1_or = squeeze(mag1_or);       %bode returns a 3D array, squeeze returns array without array with dimension 1
mag2_or = squeeze(mag2_or); 
ph1_or = squeeze(ph1_or); 
ph2_or = squeeze(ph2_or);

figure('name','Bodeplot of encoder1 without PI using calculations')
subplot(2,1,1)
loglog(w1,mag1_or),grid
subplot(2,1,2)
semilogx(w1,ph1_or),grid

% design procedure PI encoder 1
phi_1 = -(180 - delta_phi - add_phase_lag)
omega_co1 = w1(find(ph1_or < phi_1, 1))
%omega_co1 = interp1(ph1_or,w1,phi_1)

T_i1 = 1/(omega_co1*tan(add_phase_lag * pi /180))

num_I = [T_i1,1];
den_I = [T_i1,0];

sys_I = tf(num_I,den_I);

figure('name','Bodeplot integrator I')
bode(sys_I)

num_enc1_I = conv(num_enc1_or,num_I);
den_enc1_I = conv(den_enc1_or,den_I);

sys_enc1_I = tf(num_enc1_I,den_enc1_I);

[mag1_I,ph1_I,w1_I] = bode(sys_enc1_I);

mag1_I = squeeze(mag1_I); 
ph1_I = squeeze(ph1_I); 

figure('name','Bodeplot encoder 1 with I')
bode(sys_enc1_I)

% omega_co1_I = interp1(mag1_I,w1_I,1)
mag1_omega_co = interp1(w1_I,mag1_I,omega_co1)

k_I = 1/mag1_omega_co


% Calculating PI controller
num_enc1_PI = k_I*num_enc1_I;
den_enc1_PI = den_enc1_I;

sys_enc1_PI = tf(num_enc1_PI,den_enc1_PI);

[mag1_PI,ph1_PI,w1_PI] = bode(sys_enc1_PI);

mag1_PI = squeeze(mag1_PI); 
ph1_PI = squeeze(ph1_PI); 

figure('name','Bodeplot encoder 1 with PI')
bode(sys_enc1_PI),grid