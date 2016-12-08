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

sys_pos_1_close_c = feedback(sys_pos_1_open_c,1);
sys_pos_1_close_d = feedback(sys_pos_1_open_d,1);

sys_pos_2_close_c = feedback(sys_pos_2_open_c,1);
sys_pos_2_close_d = feedback(sys_pos_2_open_d,1);

if show_figures5 == 1
    figure('name','closed loop bode diagram position continuous time')
    bode(sys_pos_1_close_c)
    hold on
    bode(sys_pos_2_close_c)
    hold off
    legend('position 1', 'position 2')
end

sys_pos_1_open_d = c2d(sys_pos_1_open_c,Ts);
sys_pos_2_open_d = c2d(sys_pos_2_open_c,Ts);

if show_figures5 == 1
    figure('name','closed loop bode diagram position discrete time')
    bode(sys_pos_1_close_d)
    hold on
    bode(sys_pos_2_close_d)
    hold off
    legend('position 1', 'position 2')
end

if show_figures5 == 1
    figure('name','Root Locus of open loop position continuous time')
    subplot(1,2,1)
    rlocus(sys_pos_1_open_c)
    subplot(1,2,2)
    rlocus(sys_pos_2_open_c)
end

%% Designing proportional controller

% From reading rlocus plot manually
K_pos1 = 4.33;      % Gives damping ratio 0.7 and frequency 33.8
K_pos2 = 11.5;      % Gives damping ratio 0.7 and frequency 65.1
K = max(K_pos1, K_pos2);% Select same K for both systems so that they have the same steady state value

% Or from setting the steady state error
e_ss_des = 0.05;
K = (1-e_ss_des)/(e_ss_des*evalfr(sys_pos_1_open_c,1e-5));  % should be evaluated at freqency 0, but Matlab doesn't like that

proportional_1 = tf([K],[1]);
proportional_2 = tf([K],[1]);

% With proportional controller
sys_pos_1_open_comp_c = series(proportional_1, sys_pos_1_open_c);
sys_pos_2_open_comp_c = series(proportional_2, sys_pos_2_open_c);

sys_pos_1_close_comp_c = feedback(sys_pos_1_open_comp_c,1);
sys_pos_2_close_comp_c = feedback(sys_pos_2_open_comp_c,1);

if show_figures5 == 1
    figure('name','closed loop with proportional controller continuous time')
    bode(sys_pos_1_close_comp_c)
    hold on
    bode(sys_pos_2_close_comp_c)
    hold off
    legend('position 1', 'position 2')
end

if show_figures5 == 1
    figure('name','step respons with proportional controller continuous time')
    step(sys_pos_1_close_comp_c)
    hold on
    step(sys_pos_2_close_comp_c)
    hold off
    legend('position 1', 'position 2')
end

sys_pos_1_close_comp_d = c2d(sys_pos_1_close_comp_c,Ts);
sys_pos_2_close_comp_d = c2d(sys_pos_2_close_comp_c,Ts);

if show_figures5 == 1
    figure('name','step respons with proportional controller discrete time')
    step(sys_pos_1_close_comp_d)
    hold on
    step(sys_pos_2_close_comp_d)
    hold off
    legend('position 1', 'position 2')
end


%% Data needed to implement controller
% This is represented in Arduino file robot.h by 
    % num_contr_posi = numerator of transferfunction of P position controller of motor i
    % den_contr_posi = denominator of transferfunction of P position controller of motor i
sys_P_pos_1_d = c2d(proportional_1, Ts, 'zoh');
sys_P_pos_2_d = c2d(proportional_1, Ts, 'zoh');


clear show_figures5
