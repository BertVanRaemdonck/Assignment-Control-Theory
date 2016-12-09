if ~ exist('show_figures5', 'var')
    close all
    clear all
    clc
    clear
    show_figures5 = 1;   % show_figures3 can be set to 0 in another file to suppress the figures in this code
end

%% Run assignment_3_1_3.m
show_figures = 0;   % assignment_3_1_1 doesn't display its figures, if set to zero, so works a bit faster
show_figures2 = 0;  % assignment_3_1_2 doesn't display its figures, if set to zero, so works a bit faster
show_figures3 = 0;  % assignment_3_1_3 doesn't display its figures, if set to zero, so works a bit faster
show_figures4 = 0;  % assignment_3_2_1 doesn't display its figures, if set to zero, so works a bit faster
assignment_3_2_1


%% Calculating open loop
close all
clc

sys_pos1_close_c = feedback(sys_pos1_open_c,1);
sys_pos1_close_d = feedback(sys_pos1_open_d,1);

sys_pos2_close_c = feedback(sys_pos2_open_c,1);
sys_pos2_close_d = feedback(sys_pos2_open_d,1);

if show_figures5 == 1
    figure('name','closed loop bode diagram position continuous time')
    bode(sys_pos1_open_c)
    hold on
    bode(sys_pos2_open_c)
    hold off
    legend('position 1', 'position 2')
end

sys_pos1_open_d = c2d(sys_pos1_open_c,Ts);
sys_pos2_open_d = c2d(sys_pos2_open_c,Ts);

if show_figures5 == 1
    figure('name','closed loop bode diagram position discrete time')
    bode(sys_pos1_close_d)
    hold on
    bode(sys_pos2_close_d)
    hold off
    legend('position 1', 'position 2')
end

if show_figures5 == 1
    figure('name','Root Locus of open loop position continuous time')
    subplot(1,2,1)
    rlocus(sys_pos1_open_c)
    subplot(1,2,2)
    rlocus(sys_pos2_open_c)
end

%% Designing proportional controller

% From reading rlocus plot manually
K_pos1 = 4.33;      % Gives damping ratio 0.7 and frequency 33.8
K_pos2 = 11.5;      % Gives damping ratio 0.7 and frequency 65.1
K = max(K_pos1, K_pos2);% Select same K for both systems so that they have the same steady state value

% Or from setting the steady state error
e_ss_des = 0.05;
K = (1-e_ss_des)/(e_ss_des*evalfr(sys_pos1_open_c,1e-5));  % should be evaluated at freqency 0, but Matlab doesn't like that

sys_P_pos1 = tf([K],[1]);
sys_P_pos2 = tf([K],[1]);

% With proportional controller
sys_pos1_open_Pcomp_c = series(sys_P_pos1, sys_pos1_open_c);
sys_pos2_open_Pcomp_c = series(sys_P_pos2, sys_pos2_open_c);

sys_pos1_close_Pcomp_c = feedback(sys_pos1_open_Pcomp_c,1);
sys_pos2_close_Pcomp_c = feedback(sys_pos2_open_Pcomp_c,1);

if show_figures5 == 1
    figure('name','closed loop bode diagram with proportional controller continuous time')
    bode(sys_pos1_close_Pcomp_c)
    hold on
    bode(sys_pos2_close_Pcomp_c)
    hold off
    legend('position 1', 'position 2')
end

if show_figures5 == 1
    figure('name','step respons with P controller continuous time')
    step(sys_pos1_close_Pcomp_c)
    hold on
    step(sys_pos2_close_Pcomp_c)
    hold off
    legend('position 1', 'position 2')
end

sys_pos1_close_Pcomp_d = c2d(sys_pos1_close_Pcomp_c,Ts);
sys_pos2_close_Pcomp_d = c2d(sys_pos2_close_Pcomp_c,Ts);

if show_figures5 == 1
    figure('name','step respons with P controller discrete time')
    step(sys_pos1_close_Pcomp_d)
    hold on
    step(sys_pos2_close_Pcomp_d)
    hold off
    legend('position 1', 'position 2')
end


% Data needed to implement controller
% This is represented in Arduino file robot.h by 
    % num_contr_posi = numerator of transferfunction of P position controller of motor i
    % den_contr_posi = denominator of transferfunction of P position controller of motor i
sys_P_pos1_d = c2d(sys_P_pos1, Ts, 'zoh');
sys_P_pos2_d = c2d(sys_P_pos1, Ts, 'zoh');


%% Design of PI controller

PM_pos1 = 45;           % desired phase margin of controller 1
omega_co_pos1 = 50;     % desired crossover frequency in rad/s
PM_pos2 = 45;           % desired phase margin of controller 2
omega_co_pos2 = 50;     % desired crossover frequency in rad/s

% calculating magnitude and phase for needed calculations
w_pos = logspace(low_exp_w,high_exp_w,number_points_w);

[mag_pos1_c,ph_pos1_c] = bode(sys_pos1_open_c.num, sys_pos1_open_c.den, w_pos);
[mag_pos2_c,ph_pos2_c] = bode(sys_pos2_open_c.num, sys_pos2_open_c.den, w_pos);

% calculating parameters of controller
phase_PI_pos1 = -(-180 + PM_pos1 - interp1(w, ph_pos1_c, omega_co_pos1));   % phase of the PI controller at the crossover frequency for encoder 1
phase_PI_pos2 = -(-180 + PM_pos2 - interp1(w, ph_pos2_c, omega_co_pos2));   % phase of the PI controller at the crossover frequency for encoder 2

T_i_pos1 = tan(pi/180*(90 - phase_PI_pos1))/omega_co_pos1;          % time constant of PI encoder 1
T_i_pos2 = tan(pi/180*(90 - phase_PI_pos2))/omega_co_pos2;          % time constant of PI encoder 2
mag_PI_co_pos1 = abs(1/(j*omega_co_pos1)*(j*omega_co_pos1+1/T_i_pos1)); % magnitude of PI controller of encoder 1 at crossover frequency
mag_PI_co_pos2 = abs(1/(j*omega_co_pos2)*(j*omega_co_pos2+1/T_i_pos2)); % magnitude of PI controller of encoder 2 at crossover frequency
K_pos1 = 1/(mag_PI_co_pos1 * interp1(w, mag_pos1_c, omega_co_pos1));   % proportional constant of PI controller encoder 1
K_pos2 = 1/(mag_PI_co_pos2 * interp1(w, mag_pos2_c, omega_co_pos2));   % proportional constant of PI controller encoder 2

num_PI_pos1 = [K_pos1*T_i_pos1 K_pos1];        % D(s) = (K * T_i * s + K)/(T_i * s)
num_PI_pos2 = [K_pos2*T_i_pos2 K_pos2];
den_PI_pos1 = [T_i_pos1 0];
den_PI_pos2 = [T_i_pos2 0];
sys_PI_pos1 = tf(num_PI_pos1, den_PI_pos1);   
sys_PI_pos2 = tf(num_PI_pos2, den_PI_pos2);  

% viewing compensated system
sys_pos1_open_PIcomp_c = series(sys_PI_pos1, sys_pos1_open_c);    % Represents total system with controller
sys_pos1_close_PIcomp_c = feedback(sys_pos1_open_PIcomp_c,1);

sys_pos2_open_PIcomp_c = series(sys_PI_pos2, sys_pos2_open_c);
sys_pos2_close_PIcomp_c = feedback(sys_pos2_open_PIcomp_c,1);

if show_figures5 == 1
    figure('name','closed loop bode diagram with PI controller continuous time')
    bode(sys_pos1_close_PIcomp_c)
    hold on
    bode(sys_pos2_close_PIcomp_c)
    hold off
    legend('position 1', 'position 2')
    
    figure('name','step respons with PI controller continuous time')
    step(sys_pos1_close_PIcomp_c)
    hold on
    step(sys_pos2_close_PIcomp_c)
    hold off
    legend('position 1', 'position 2')
end

% data needed to implement controller on Arduino
sys_PI_pos1_d = c2d(sys_PI_pos1, Ts, 'zoh');
sys_PI_pos2_d = c2d(sys_PI_pos2, Ts, 'zoh');


clear show_figures5
