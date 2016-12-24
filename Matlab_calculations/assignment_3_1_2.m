if ~ exist('show_figures2', 'var')
    close all
    clear all
    clc
    clear
    show_figures2 = 1;   % show_figures2 can be set to 0 in another file to suppress the figures in this code
end

%% Assignment_3_1_1.m opvragen
show_figures = 0;   % assignment_3_1_1 doesn't display its figures, if set to zero, so works a bit faster
assignment_3_1_1

set(0,'defaultlinelinewidth',2)
set(0,'defaultaxesfontsize',12)


%% Parameters, getting data and calculations

close all
clc

% parameters
low_exp_w = -1;     % w = logspace(low_exp_w,high_exp_w,number_points_w)
high_exp_w = 3;
number_points_w = 1000;

nb_speed_ref = 300;    % number of points of vector with the reference speed it has to track
mag_speed_ref = 4000;   % max magnitude of vector with the reference speed it has to track
speed_ref = (mag_speed_ref/4)*ones([1,nb_speed_ref/3]);     % used for the first simulations of the controller, 
speed_ref = [speed_ref  (mag_speed_ref/2)*ones([1,nb_speed_ref/3])];    % not for comparison with the reality
speed_ref = [speed_ref  (mag_speed_ref/1)*ones([1,nb_speed_ref/3])];

PM1 = 55;           % desired phase margin of controller 1
omega_co1 = 15;     % desired crossover frequency in rad/s -> higher than this won't decrease tr much because saturation
PM2 = 55;           % desired phase margin of controller 2
omega_co2 = 15;     % desired crossover frequency in rad/s

% Taking data from assignment_3_1_1.m
num_enc1_or = num_enc1;
den_enc1_or = den_enc1;
num_enc2_or = num_enc2;
den_enc2_or = den_enc2;

sys_enc1_or = tf(num_enc1_or,den_enc1_or,Ts);
sys_enc2_or = tf(num_enc2_or,den_enc2_or,Ts);

sys_enc1_c = d2c(sys_enc1_or);    % We have to work in continuous time
sys_enc2_c = d2c(sys_enc2_or);

if show_figures2 == 1
    figure('name','Bodeplot of encoders without PI in discrete and continuous time')
    subplot(2,1,1)
    bode(sys_enc1_or, sys_enc1_c),grid
    legend('enc1 disc', 'enc1 cont','location','southwest')
    title('Bode diagram for encoder 1')
    subplot(2,1,2)
    bode(sys_enc2_or, sys_enc2_c),grid
    legend('enc2 disc', 'enc2 cont','location','southwest')
    title('Bode diagram for encoder 2')
end

% calculating magnitude and phase for needed calculations
w = logspace(low_exp_w,high_exp_w,number_points_w);

[mag1_c,ph1_c] = bode(sys_enc1_c.num, sys_enc1_c.den, w);
[mag2_c,ph2_c] = bode(sys_enc2_c.num, sys_enc2_c.den, w);



%% Design procedure PI's

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

if show_figures2 == 1
    figure('name', 'Bodeplot of the PI compensators')
    subplot(2,1,1)
    bode(sys_PI1, w), grid
    title('Bode diagram PI for encoder 1')
    subplot(2,1,2)
    bode(sys_PI2, w), grid
    title('Bode diagram PI for encoder 2')
end

%% Viewing compensated systems

sys_enc1_PIcomp_c = series(sys_PI1, sys_enc1_c);    % Represents total system with controller
sys_enc1_PIcomp_d = c2d(sys_enc1_PIcomp_c,Ts);      % We work in discrete time

sys_enc2_PIcomp_c = series(sys_PI2, sys_enc2_c);
sys_enc2_PIcomp_d = c2d(sys_enc2_PIcomp_c,Ts);

if show_figures2 == 1
    figure('name', 'Bodeplot of the compensated systems in continuous time')
    subplot(2,1,1)
    bode(sys_enc1_c, sys_enc1_PIcomp_c, w), grid
    title('Bode diagram for the compensated encoder 1')
    legend('enc1', 'enc1_PIcomp','location','southwest')
    subplot(2,1,2)
    bode(sys_enc2_c, sys_enc2_PIcomp_c, w), grid
    title('Bode diagram for the compensated encoder 2')
    legend('enc2', 'enc2_PIcomp','location','southwest')

    figure('name', 'Bodeplot of the compensated systems in discrete time')
    subplot(2,1,1)
    bode(sys_enc1_or, sys_enc1_PIcomp_d, w), grid
    legend('enc1', 'enc1_PIcomp','location','southwest')
    title('Bode diagram for the compensated encoder 1')
    subplot(2,1,2)
    bode(sys_enc2_or, sys_enc2_PIcomp_d, w), grid
    legend('enc2', 'enc2_PIcomp','location','southwest')
    title('Bode diagram for the compensated encoder 2')
end


%% Testing controller with simulations

% Making feedback loop
sys_enc1_or_fb = feedback(sys_enc1_or,1);   % assumes perfect measurements => unity feedback
sys_enc2_or_fb = feedback(sys_enc2_or,1);

sys_enc1_PIcomp_fb = feedback(sys_enc1_PIcomp_d,1); % assumes perfect measurements => unity feedback
sys_enc2_PIcomp_fb = feedback(sys_enc2_PIcomp_d,1);

% Time simulation of system encoders with controller
time_fb =0:Ts:(length(speed_ref)-1)*Ts;
y1_enc1_or_fb = lsim(sys_enc1_or_fb,speed_ref,time_fb);
y1_enc1_PIcomp_fb = lsim(sys_enc1_PIcomp_fb,speed_ref,time_fb,'--'); 

y1_enc2_or_fb = lsim(sys_enc2_or_fb,speed_ref,time_fb);
y1_enc2_PIcomp_fb = lsim(sys_enc2_PIcomp_fb,speed_ref,time_fb,'--');    

if show_figures2 == 1
    figure('name','lsim encoders with feedback loop')
    subplot(2,1,1)
    plot(time_fb,speed_ref)
    grid on
    hold on
    plot(time_fb,y1_enc1_or_fb,':')
    plot(time_fb,y1_enc1_PIcomp_fb,'--')
    xlabel('t [s]')
    ylabel('\omega_1 [enc/s]')
    legend('reference input','without controller','with controller','location','northwest')
    title('encoder 1')
    hold off
    subplot(2,1,2)
    plot(time_fb,speed_ref)
    grid on
    hold on
    plot(time_fb,y1_enc2_or_fb,':')
    plot(time_fb,y1_enc2_PIcomp_fb,'--')
    xlabel('t [s]')
    ylabel('\omega_2 [enc/s]')
    legend('reference input','without controller','with controller','location','northwest')
    title('encoder 2')
    hold off
end

%% Data needed to implement controller
% This is represented in Arduino file robot.h by 
    % num_contr_speedi = numerator of transferfunction of PI controller of motor i
    % den_contr_speedi = denominator of transferfunction of PI controller of motor i
sys_PI1_d = c2d(sys_PI1, Ts, 'zoh');
sys_PI2_d = c2d(sys_PI2, Ts, 'zoh');


% %% Custom simulation of time response of compensated system
% % To test if the implemented code in Arduino gives logical results
% % Used for debugging
% t = 0:Ts:30;          % [s]
% ek_speed1 = [0.0, 0.0];
% uk_speed1 = [0.0, 0.0];
% 
% ek = [zeros(1, round(length(t)/3)), -100*ones(1, length(t)-round(length(t)/3))];
% uk = zeros(size(ek));
% 
% den_contr_speed1 = cell2mat(sys_PI1_d.den);
% num_contr_speed1 = cell2mat(sys_PI2_d.num);
% 
% for time_index = 1:length(ek)
%     % shift memories
%     memory_index = 1;
%     while j > 0
%         ek_speed1(memory_index+1) = ek_speed1(memory_index+1-1);   % the j+1 is there because c++ starts from 0 but matlab from 1
%         uk_speed1(memory_index+1) = uk_speed1(memory_index+1-1);
%         memory_index = memory_index-1;
%     end
%     ek_speed1(1) = ek(time_index);
%     
%     % compute new voltage
%     uk_speed1(0+1) = 1/(den_contr_speed1(0+1)) * ...
%                      (-den_contr_speed1(1+1)*uk_speed1(1+1) + ...
%                      num_contr_speed1(0+1)*ek_speed1(0+1) + ...
%                      num_contr_speed1(1+1)*ek_speed1(1+1));
%     
%     % clip output of the controllers
%     if uk_speed1(0+1) > 6000
%         uk_speed1(0+1) = 6000;
%     end
%     if uk_speed1(0+1) < -6000
%         uk_speed1(0+1) = -6000;
%     end
%     
%     % put control signal in vector for viewing
%     uk(time_index) = uk_speed1(1);
%     
% end
% 
% if show_figures2 == 1
%     figure()
%     plot(t, ek)
%     hold on
%     plot(t, uk)
%     hold off
% end


%% Comparison simulation of speed motors with actual response
% clipping data to get limited time view
t_start = 0 * period * 1e-3;
t_stop = 5 * period * 1e-3;

% compile file name and import data
type = 'block';         % type of the signal: 'block', 'rand', ...
period = 1000;          % period of the signal, 0 if it isn't periodical

if period > 0
    log_name = sprintf('log_gpio_vc_%s_(%d).xml', type, period);
else
    log_name = sprintf('log_gpio_vc_%s.xml', type);
end
rec = readlog(log_name);

period = period + 1;

% raw input data - these are sampled at a non-uniform rate!

t_input = rec.getData('time');
speed1_des = rec.getData('speed1_des');
speed1_act = rec.getData('speed1_act');
control_signal1 = rec.getData('control_signal1');
speed2_des = rec.getData('speed2_des');
speed2_act = rec.getData('speed2_act');
control_signal2 = rec.getData('control_signal2');
v_input = rec.getData('voltage');
encoder1 = rec.getData('encoder1');

% Unwrapping the values of the encoder
enc_bits = 16;  % amount of bits used by the encoder
encoder1 = cust_unwrap(encoder1, enc_bits);

% Interpolate the input data to uniform timesteps 
t = (t_input(1):Ts*1e3:t_input(1)+(length(t_input)-1)*Ts*1e3)'; % the equivalent of t_input if Ts were truly uniform
v = v_input;                                                    % v doesn't have to be interpolated because the Arduino does a zoh                  
speed1_des = interp1(t_input,speed1_des,t);                     % values of desired speed of motor 1 at the corresponding times
speed1_act = interp1(t_input,speed1_act,t);                     % values of actual speed of motor 1 at the corresponding times
control_singal1 = interp1(t_input,control_signal1,t);           % values of control signal of motor 1 at the corresponding times
speed2_des = interp1(t_input,speed2_des,t);                     % values of desired speed of motor 2 at the corresponding times
speed2_act = interp1(t_input,speed2_act,t);                     % values of actual speed of motor 2 at the corresponding times
control_singal2 = interp1(t_input,control_signal2,t);           % values of control signal of motor 2 at the corresponding times
encoder1 = interp1(t_input,encoder1,t);                         % values of the difference in encoder value at the corresponging times
t = 1e-3*(t - t_input(1));                                      % set t to be in s and start at 0 for convenience

% Calculating simulations
speed1_PIcomp_fb = lsim(sys_enc1_PIcomp_fb,speed1_des,t,':');
speed2_PIcomp_fb = lsim(sys_enc2_PIcomp_fb,speed2_des,t,':');


% Plotting data speed motor 1
if show_figures2 == 1
    figure('name', 'Comparison speed motor 1')
    subplot(2,2,1)
    plot(t, speed1_des,'--');
    grid on
    hold on
    plot(t, speed1_act);
    hold off
    xlabel('t [s]')
    ylabel('\omega_1 [enc/s]')
    axis([t_start t_stop -inf inf]);
    legend('desired speed','actual speed','location','northwest')
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
    legend('desired speed','simulated speed','location','northwest')
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
    legend('error actual','error simulated')
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
if show_figures2 == 1
    figure('name', 'Comparison speed motor 2')
    subplot(2,2,1)
    plot(t, speed2_des,'--');
    grid on
    hold on
    plot(t, speed2_act);
    hold off
    xlabel('t [s]')
    ylabel('\omega_2 [enc/s]')
    axis([t_start t_stop -inf inf]);
    legend('desired speed','actual speed','location','northwest')
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
    legend('desired speed','simulated speed','location','northwest')
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


clear show_figures2