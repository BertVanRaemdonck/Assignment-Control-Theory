if ~ exist('show_figures5', 'var')
    close all
    clear all
    clc
    clear
    show_figures5 = 1;   % show_figures3 can be set to 0 in another file to suppress the figures in this code
end

%% Assignment_3_1_3.m opvragen
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

figure()
step(sys_pos_1_close_comp_c)
hold on
step(sys_pos_2_close_comp_c)
hold off
legend('position 1', 'position 2')


%% Probeersel

if show_figures5 == 1
    figure('name','Step respons without proportional controller discrete time')
    step(sys_pos_1_close_d)
    hold on
    step(sys_pos_2_close_d)
    hold off
    legend('position 1', 'position 2')
end

if show_figures5 == 1
    figure('name','Root Locus of open loop position discrete time')
    subplot(1,2,1)
    rlocus(sys_pos_1_open_d)
    subplot(1,2,2)
    rlocus(sys_pos_2_open_d)
end


% From reading rlocus plot manually

K_pos1_d = 4.73;      % Gives damping ratio 0.712 and frequency 38.3
K_pos2_d = 15.1;      % Gives damping ratio 0.708 and frequency 99.4

proportional_1_d = tf([K_pos1_d],[1]);
proportional_2_d = tf([K_pos2_d],[1]);

% With proportional controller
sys_pos_1_close_comp_d = feedback(series(proportional_1_d,sys_pos_1_open_d),1);
sys_pos_2_close_comp_d = feedback(series(proportional_2_d,sys_pos_2_open_d),1);

if show_figures5 == 1
    figure('name','closed loop with proportional controller discrete time')
    bode(sys_pos_1_close_comp_d)
    hold on
    bode(sys_pos_2_close_comp_d)
    hold off
    legend('position 1', 'position 2')
end

if show_figures5 == 1
    figure('name','Step respons with proportional controller discrete time')
    step(sys_pos_1_close_comp_d)
    hold on
    step(sys_pos_2_close_comp_d)
    hold off
    legend('position 1', 'position 2')
end

clear show_figures5
