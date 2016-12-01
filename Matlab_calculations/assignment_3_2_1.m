if ~ exist('show_figures4', 'var')
    close all
    clear all
    clc
    clear
    show_figures4 = 1;   % show_figures3 can be set to 0 in another file to suppress the figures in this code
end

%% Assignment_3_1_3.m opvragen
show_figures = 0;   % assignment_3_1_1 doesn't display its figures, if set to zero, so works a bit faster
show_figures2 = 0;  % assignment_3_1_2 doesn't display its figures, if set to zero, so works a bit faster
show_figures3 = 0;  % assignment_3_1_3 doesn't display its figures, if set to zero, so works a bit faster
assignment_3_1_3


%% Calculating open loop
close all
clc

sys_enc1_PIcomp_fb_c = feedback(sys_enc1_PIcomp_c,1);   % Feedback system of speed 1 in continuous time
sys_enc2_PIcomp_fb_c = feedback(sys_enc2_PIcomp_c,1);   % Feedback system of speed 2 in continuous time

differentiator = tf([1,0],[0,1]);   % differentiator = s
integrator = tf([0,1],[1,0]);       % integrator = 1/s

sys_pos_1_open_c = series(differentiator,series(sys_enc1_PIcomp_fb_c,integrator));    % open loop system of position 1
sys_pos_2_open_c = series(differentiator,series(sys_enc2_PIcomp_fb_c,integrator));    % open loop system of position 2

if show_figures4 == 1
    figure('name','openloop bode diagram positie 1 continuous time')
    bode(sys_pos_1_open_c)
end

sys_pos_1_open_d = c2d(sys_pos_1_open_c, Ts);
sys_pos_2_open_d = c2d(sys_pos_2_open_c, Ts);

if show_figures4 == 1
    figure('name','openloop bode diagram positie 1 discrete time')
    bode(sys_pos_1_open_d)
end



clear show_figures4

