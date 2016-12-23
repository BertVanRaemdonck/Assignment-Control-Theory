if ~ exist('show_figures3', 'var')
    close all
    clear all
    clc
    clear
    show_figures3 = 1;   % show_figures3 can be set to 0 in another file to suppress the figures in this code
end


%% Assignment_3_1_2.m opvragen
show_figures = 0;   % assignment_3_1_1 doesn't display its figures, if set to zero, so works a bit faster
show_figures2 = 0;  % assignment_3_1_2 doesn't display its figures, if set to zero, so works a bit faster
assignment_3_1_2

set(0,'defaultlinelinewidth',2)
set(0,'defaultaxesfontsize',12)

%% Parameters, getting data and calculations
close all
clc

% no parameters of the controller are adapted, so we use the same names as
% in the previous assignment.

% compile file name and import data
rec = readlog('log_gpio_vc_incl_up.xml');

% raw input data - these are sampled at a non-uniform rate!

t_input = rec.getData('time');
speed1_des = rec.getData('speed1_des');
speed1_act = rec.getData('speed1_act');
control_signal1 = rec.getData('control_signal1');
speed2_des = rec.getData('speed2_des');
speed2_act = rec.getData('speed2_act');
control_signal2 = rec.getData('control_signal2');
v_input = rec.getData('voltage');


% Interpolate the input data to uniform timesteps 
t = (t_input(1):Ts*1e3:t_input(1)+(length(t_input)-1)*Ts*1e3)'; % the equivalent of t_input if Ts were truly uniform
v = v_input;                                                    % v doesn't have to be interpolated because the Arduino does a zoh                  
speed1_des = interp1(t_input,speed1_des,t);                     % values of desired speed of motor 1 at the corresponding times
speed1_act = interp1(t_input,speed1_act,t);                     % values of actual speed of motor 1 at the corresponding times
control_singal1 = interp1(t_input,control_signal1,t);           % values of control signal of motor 1 at the corresponding times
speed2_des = interp1(t_input,speed2_des,t);                     % values of desired speed of motor 2 at the corresponding times
speed2_act = interp1(t_input,speed2_act,t);                     % values of actual speed of motor 2 at the corresponding times
control_singal2 = interp1(t_input,control_signal2,t);           % values of control signal of motor 2 at the corresponding times
t = 1e-3*(t - t_input(1));                                      % set t to be in s and start at 0 for convenience

%% Plotting data of velocity with constant force disturbance
% The cart was set on an inclination and both motors were set to the same
% velocity, in order to let the cart drive straight forward.

% Calculating simulations
speed1_PIcomp_fb = lsim(sys_enc1_PIcomp_fb,speed1_des,t,':');
speed2_PIcomp_fb = lsim(sys_enc2_PIcomp_fb,speed2_des,t,':');

% clipping data to get limited time view
t_start = 0;
t_stop = 5;

% Plotting data speed motor 1
if show_figures3 == 1
    figure('name', 'Comparison speed motor 1 with force disturbance')
    subplot(2,2,1)
    plot(t, speed1_des,'--');
    grid on
    hold on
    plot(t, speed1_act);
    hold off
    xlabel('t [s]')
    ylabel('\omega_1 [enc/s]')
    axis([t_start t_stop -inf inf]);
    legend('desired speed','actual speed','location','northeast')
    title('speed comparison of motor 1')
    subplot(2,2,2)
    plot(t, speed1_des,'--');
    grid on
    hold on
    plot(t, speed1_PIcomp_fb);
    hold off
    xlabel('t [s]')
    ylabel('\omega_1 [enc/s]')
    axis([t_start t_stop -inf inf]);
    legend('desired speed','simulated speed','location','southeast')
    title('speed comparison of motor 1')
    subplot(2,2,3)
    plot(t, speed1_act-speed1_des);
    grid on
    hold on
    plot(t, speed1_PIcomp_fb-speed1_des,'--');
    hold off
    xlabel('t [s]')
    ylabel('\Delta \omega_1 [enc/s]')
    axis([t_start t_stop -inf inf]);
    legend('error actual','error simulated','location','southeast')
    title('error between speed and desired speed')
    subplot(2,2,4)
    plot(t, control_signal1);
    grid on
    xlabel('t [s]')
    ylabel('V_1 [mV]')
    axis([t_start t_stop -inf inf]);
    title('control signal of motor 1')
end

% Plotting data speed motor 2
if show_figures3 == 1
    figure('name', 'Comparison speed motor 2 with force disturbance')
    subplot(2,2,1)
    plot(t, speed2_des,'--');
    grid on
    hold on
    plot(t, speed2_act);
    hold off
    xlabel('t [s]')
    ylabel('\omega_2 [enc/s]')
    axis([t_start t_stop -inf inf]);
    legend('desired speed','actual speed','location','southeast')
    title('speed comparison of motor 2')
    subplot(2,2,2)
    plot(t, speed2_des,'--');
    grid on
    hold on
    plot(t, speed2_PIcomp_fb);
    hold off
    xlabel('t [s]')
    ylabel('\omega_2 [enc/s]')
    axis([t_start t_stop -inf inf]);
    legend('desired speed','simulated speed','location','northeast')
    title('speed comparison of motor 2')
    subplot(2,2,3)
    plot(t, speed2_act-speed2_des);
    grid on
    hold on
    plot(t, speed2_PIcomp_fb-speed2_des,'--');
    hold off
    xlabel('t [s]')
    ylabel('\Delta \omega_2 [enc/s]')
    axis([t_start t_stop -inf inf]);
    legend('error actual','error simulated')
    title('error between speed and desired speed')
    subplot(2,2,4)
    plot(t, control_signal2);
    grid on
    xlabel('t [s]')
    ylabel('V_2 [mV]')
    axis([t_start t_stop -inf inf]);
    title('control signal of motor 2')
end


clear show_figures3
