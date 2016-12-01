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

sys_pos_1_open_c_fb = feedback(sys_pos_1_open_c,1);
sys_pos_1_open_d_fb = feedback(sys_pos_1_open_d,1);

if show_figures5 == 1
    figure('name','closed loop bode diagram position 1 continuous time')
    bode(sys_pos_1_open_c_fb)
end

sys_pos_1_open_d = c2d(sys_pos_1_open_c,Ts);

if show_figures5 == 1
    figure('name','closed loop bode diagram position 1 discrete time')
    bode(sys_pos_1_open_d_fb)
end

figure()
subplot(1,2,1)
rlocus(sys_pos_1_open_c)
subplot(1,2,2)
rlocus(sys_pos_2_open_c)



clear show_figures5
