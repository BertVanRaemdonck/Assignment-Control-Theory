if ~ exist('show_figures6', 'var')
    close all
    clear all
    clc
    clear
    show_figures6 = 1;   % show_figures6 can be set to 0 in another file to suppress the figures in this code
end

%% Run assignment_3_1_3.m
show_figures = 0;   % assignment_3_1_1 doesn't display its figures, if set to zero, so works a bit faster
show_figures2 = 0;  % assignment_3_1_2 doesn't display its figures, if set to zero, so works a bit faster
show_figures3 = 0;  % assignment_3_1_3 doesn't display its figures, if set to zero, so works a bit faster
show_figures4 = 0;  % assignment_3_2_1 doesn't display its figures, if set to zero, so works a bit faster
show_figures5 = 0;  % assignment_3_2_2 doesn't display its figures, if set to zero, so works a bit faster
% assignment_3_2_2


%% Defining system of position sensor fusion
close all
clc

Ts = 1/100;

% x = [d; s]|k      state vector
    % d = initial distance to wall
    % s = relative travelled distance
% u = [v]|k         input vector
    % v = speed of the cart
% y = output vector = distance to the wall at this moment

A_fusion = [1,0;
            0,1];
B_fusion = [0;
            Ts];
C_fusion = [1,1];
D_fusion = [0];

sys_fusion_d = ss(A_fusion,B_fusion,C_fusion,D_fusion,Ts);

% Calculating the poles of the system
 
poles_fusion = eig(A_fusion)

if show_figures6 == 1
    figure('name','Root locus of discrete system position fusion')
    rlocus(sys_fusion_d)
end

%% Placing new poles
% This is not possible, the system is uncontrollable due to the fact that
% the poles are on the edge of the stability region.
% We make a open loop estimator in the Arduino files.

% OR we check the controllability matrix

n_controllability_matrix = 2;
controllability_matrix = [B_fusion  A_fusion*B_fusion];
rank_controllability_matrix = rank(controllability_matrix);

if n_controllability_matrix ~= rank_controllability_matrix
    disp('The matrix is not controllable')
end

if n_controllability_matrix == rank_controllability_matrix
    disp('The matrix is controllable, update the file')
end


% OR we check the observability matrix

n_observability_matrix = 2;
observability_matrix = [C_fusion  C_fusion*A_fusion];
rank_observability_matrix = rank(observability_matrix);

if n_observability_matrix ~= rank_observability_matrix
    disp('The matrix is not observable')
end

if n_observability_matrix == rank_observability_matrix
    disp('The matrix is observable, update the file')
end


%% Getting data and calculations normal position fusion

% compile file name and import data
rec = readlog('log_gpio_position_fusion(-1000)_beter.xml');

% raw input data - these are sampled at a non-uniform rate!

t_input = rec.getData('time');
initial_distance = rec.getData('initial_distance');
relative_distance = rec.getData('relative_distance');
speed_cart = rec.getData('speed_cart');
absolute_distance = rec.getData('absolute distance');
real_distance = rec.getData('real_distance');
error_distance = rec.getData('error_distance');
enc1_curr_m = rec.getData('enc1_curr_m');
enc1_prev_m = rec.getData('enc1_prev_m');
v_input = rec.getData('Voltage');


% Interpolate the input data to uniform timesteps 
t = (t_input(1):Ts*1e3:t_input(1)+(length(t_input)-1)*Ts*1e3)'; % the equivalent of t_input if Ts were truly uniform
v = v_input;                                                    % v doesn't have to be interpolated because the Arduino does a zoh                  
initial_distance = interp1(t_input,initial_distance,t);         % values of initial distance at the corresponding times, should stay the same
relative_distance = interp1(t_input,relative_distance,t);       % values of relative_distance at the corresponding times = state x
speed_cart = interp1(t_input,speed_cart,t);                     % values of the speed of the cart at the corresponding times = input u
absolute_distance = interp1(t_input,absolute_distance,t);       % values of absolute distance at the corresponding times = output y
real_distance = interp1(t_input,real_distance,t);               % values of real distance at the corresponding times = output of front sensor
error_distance = interp1(t_input,error_distance,t);             % values of the error between real and absolute distance at the corresponding times
enc1_curr_m = interp1(t_input,enc1_curr_m,t);                   % values of the values of the encoder in m
enc1_prev_m = interp1(t_input,enc1_prev_m,t);                   % previous values of the encoder in m
t = 1e-3*(t - t_input(1));                                      % set t to be in s and start at 0 for convenience

% plotting data:

t_start = 2;
t_stop = 10;

if show_figures6 == 1
    figure('name','position fusion no disturbance')
    subplot(2,2,1)
    plot(t, relative_distance);
    xlabel('t [s]')
    ylabel('state x [m]')
    axis([t_start t_stop -inf inf]);
    %legend('desired speed','actual speed','location','northeast')
    title('relative distance')
    subplot(2,2,2)
    plot(t, absolute_distance,'--');
    hold on
    plot(t, real_distance);
    hold off
    xlabel('t [s]')
    ylabel('distance [m]')
    axis([t_start t_stop -inf inf]);
    legend('absolute distance','real distance','location','northeast')
    title('distance comparison')
    subplot(2,2,3)
    plot(t, error_distance);
    xlabel('t [s]')
    ylabel('error [m]')
    axis([t_start t_stop -inf inf]);
    %legend('error actual','error simulated','location','southeast')
    title('error between real and absolute distance')
    subplot(2,2,4)
    plot(t, speed_cart);
    xlabel('t [s]')
    ylabel('speed cart [m/s]')
    axis([t_start t_stop -inf inf]);
    title('speed of the cart')
end


%% Getting data and calculations position fusion with deliberate error

% compile file name and import data
rec = readlog('log_gpio_position_fusion(-1000)_with_error.xml');

% raw input data - these are sampled at a non-uniform rate!

t_input = rec.getData('time');
initial_distance = rec.getData('initial_distance');
relative_distance = rec.getData('relative_distance');
speed_cart = rec.getData('speed_cart');
absolute_distance = rec.getData('absolute distance');
real_distance = rec.getData('real_distance');
error_distance = rec.getData('error_distance');
enc1_curr_m = rec.getData('enc1_curr_m');
enc1_prev_m = rec.getData('enc1_prev_m');
v_input = rec.getData('Voltage');


% Interpolate the input data to uniform timesteps 
t = (t_input(1):Ts*1e3:t_input(1)+(length(t_input)-1)*Ts*1e3)'; % the equivalent of t_input if Ts were truly uniform
v = v_input;                                                    % v doesn't have to be interpolated because the Arduino does a zoh                  
initial_distance = interp1(t_input,initial_distance,t);         % values of initial distance at the corresponding times, should stay the same
relative_distance = interp1(t_input,relative_distance,t);       % values of relative_distance at the corresponding times = state x
speed_cart = interp1(t_input,speed_cart,t);                     % values of the speed of the cart at the corresponding times = input u
absolute_distance = interp1(t_input,absolute_distance,t);       % values of absolute distance at the corresponding times = output y
real_distance = interp1(t_input,real_distance,t);               % values of real distance at the corresponding times = output of front sensor
error_distance = interp1(t_input,error_distance,t);             % values of the error between real and absolute distance at the corresponding times
enc1_curr_m = interp1(t_input,enc1_curr_m,t);                   % values of the values of the encoder in m
enc1_prev_m = interp1(t_input,enc1_prev_m,t);                   % previous values of the encoder in m
t = 1e-3*(t - t_input(1));                                      % set t to be in s and start at 0 for convenience

% plotting data:

t_start = 2;
t_stop = 9.5;

if show_figures6 == 1
    figure('name','position fusion with disturbance')
    subplot(2,2,1)
    plot(t, relative_distance);
    xlabel('t [s]')
    ylabel('state x [m]')
    axis([t_start t_stop -inf inf]);
    %legend('desired speed','actual speed','location','northeast')
    title('relative distance')
    subplot(2,2,2)
    plot(t, absolute_distance,'--');
    hold on
    plot(t, real_distance);
    hold off
    xlabel('t [s]')
    ylabel('distance [m]')
    axis([t_start t_stop -inf inf]);
    legend('absolute distance','real distance','location','northeast')
    title('distance comparison')
    subplot(2,2,3)
    plot(t, error_distance);
    xlabel('t [s]')
    ylabel('error [m]')
    axis([t_start t_stop -inf inf]);
    %legend('error actual','error simulated','location','southeast')
    title('error between real and absolute distance')
    subplot(2,2,4)
    plot(t, speed_cart);
    xlabel('t [s]')
    ylabel('speed cart [m/s]')
    axis([t_start t_stop -inf inf]);
    title('speed of the cart')
end


clear show_figures6
