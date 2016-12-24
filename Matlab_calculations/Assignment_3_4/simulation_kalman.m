clear all; close all; clc;

% For figures:
set(0,'defaultlinelinewidth',2)
set(0,'defaultaxesfontsize',12)

% =========================================================================
% Example simulation file for Kalman filtering on 2wd cart
% December 2016 - Ruben Van Parys
% =========================================================================

% Fill in the parameters + the functions
%   * f.m
%   * h.m
%   * Jf.m
%   * Jh.m
%   * local_error.m
%
% You can use the function drawellipsoids to plot the confidence ellipsoids
% of the estimated position.

% ============================== Parameters ===============================
% number of states
n_x = 3;
% number of system inputs
n_u = 2;
% number of measurements
n_y = 2;
% covariance of simulated process noise
Q_sim = 3e-11*eye(n_x);
% covariance of simulated measurement noise
R_sim = 8e-10*eye(n_y);
% covariance of modeled process noise
Q_mod = 3e-11*eye(n_x);
% covariance of simulated measurement noise
R_mod = 8e-10*eye(n_y);
% initial state
x0 = zeros(n_x,1);
% initial state estimate
x_est0 = zeros(n_x,1);
% feedback values
kx = 100*.5;
ky = 5000*.5;
kt = 10*12;
% trajectory file
filename = 'TrajectoryKalmanExercise.txt';

% ============================== Simulation ===============================
dead_reckoning = 0; % set to 1 to see the result with dead reckoning
state_feedback = 1; % set to 1 to enable state feedback
nb_walls = 2;       % can be set to 1 or 2 to give the amount of walls

% read file
data = dlmread(filename, '\t', 1, 0);
t = data(:,1)';
x_ref = data(:,2:4)';  % x [m], y [m], theta [rad]
u_ref = data(:,5:6)';  % v [m/s], omega [rad/s]
meas_valid = data(:,10)';
Ts = t(2) - t(1);
% create signals
x = zeros(n_x, length(t));
u = zeros(n_u, length(t));
y = zeros(n_y, length(t));
e = zeros(n_x, length(t));
x_est = zeros(n_x, length(t));
P_est = zeros(n_x, n_x, length(t));
P_est(:,:,1) = [10e-10 5e-10 0 ; 5e-10 10e-10 0 ; 0 0 1e-4];
nu = zeros(n_y, length(t));
S = zeros(n_y, n_y, length(t));
L = zeros(n_x, n_y, length(t));

% initial values
x(:,1) = x0;
x_est(:,1) = x_est0;

% feedback matrix
K = [kx,  0,  0;
      0, ky, kt];  
  
% walls
% wall 1: y = 0.05
a1 = 0;
b1 = 1;
c1 = 0.1;
if nb_walls == 1
    % wall 1 = wall 2 so that there is only one distinct wall
    a2 = a1;
    b2 = b1;
    c2 = c1;
else
    % wall 2: x = 0.25
    a2 = 1;
    b2 = 0;
    c2 = 0.25;
end

wall_params = [a1, b1, c1 ; a2, b2, c2];

figure('name', 'confidence ellipsoids simulation')
hold on
daspect([1 1 1])
cont = drawellipsoid(P_est(1:2,1:2,1));
plot(cont(:,1), cont(:,2))
% simulation

if state_feedback ~= 1
    u = u_ref;
end

for i = 2:length(t)
    % Plant simulation    
    x(:,i) = f(x(:,i-1), u(:,i-1), Ts) + (randn(1,n_x)*chol(Q_sim))';
    y(:,i) = h(wall_params, x(:,i)) + (randn(1,n_y)*chol(R_sim))';

    % Compute Jacobians
    A = Jf(x_est(:,i-1), u(:,i-1), Ts);
    C = Jh(wall_params, x_est(:,i-1));

    % Kalman prediction step
    x_est(:,i) = f(x_est(:,i-1), u(:,i-1), Ts);
    P_est(:,:,i) = A*P_est(:,:,i-1)*A' + Q_mod;
        
        
    if (dead_reckoning ~= 1 && meas_valid(i) == 1)
        % Kalman correction step
        nu(:,i) = y(:,i) - h(wall_params, x_est(:,i));
        S(:,:,i) = C*P_est(:,:,i)*C' + R_mod;
        L(:,:,i) = P_est(:,:,i)*C'/S(:,:,i);
        x_est(:,i) = x_est(:,i) + L(:,:,i)*nu(:,i);
        P_est(:,:,i) = (eye(n_x)-L(:,:,i)*C)*P_est(:,:,i);
        
        cont = drawellipsoid(P_est(1:2,1:2,i));
        plot(cont(:,1), cont(:,2))
    end
    if dead_reckoning == 1
        cont = drawellipsoid(P_est(1:2,1:2,i));
        plot(cont(:,1), cont(:,2))
    end

    % Control law
    if state_feedback == 1
        e(:,i) = local_error(x_ref(:,i), x_est(:,i));
        u(:,i) = u_ref(:,i) + K*e(:,i);
    end    

end
xlabel('e_x')
ylabel('e_y')
hold off

figure('name', 'simulated trajectory with kalman filter')
plot(x(1,:), x(2,:));
grid on;
hold on
plot(x_est(1,:), x_est(2,:));
hold on
plot(x_ref(1,:), x_ref(2,:));
hold off
legend('actual', 'estimated', 'reference')

figure('name', 'errors in tracking the simulated trajectory')
subplot(4,1,1)
plot(x_ref(1,:) - x_est(1,:))
ylabel('x_{ref} - x_{est} [m]')
grid on
subplot(4,1,2)
plot(x_ref(2,:) - x_est(2,:))
ylabel('y_{ref} - y_{est} [m]')
grid on
subplot(4,1,3)
plot(x_ref(3,:) - x_est(3,:))
ylabel('\theta_{ref} - \theta_{est} [rad]')
grid on
subplot(4,1,4)
plot(meas_valid(:))
ylabel({'measurement','valid'})
grid on


% ============================== Experiment 1 ===============================

rec = readlog('log_gpio_feedback_test6.xml');

t_input = rec.getData('time');
vA = rec.getData('wheel_speedA');
vB = rec.getData('wheel_speedB');
x_ref = rec.getData('x_ref');
y_ref = rec.getData('y_ref');
theta_ref = rec.getData('theta_ref');
x_kalman = rec.getData('x_kalman');
y_kalman = rec.getData('y_kalman');
theta_kalman = rec.getData('theta_kalman');
meas_valid = rec.getData('meas_valid');

figure('name', 'comparison states experiment 1')
subplot(4,1,1)
plot(x_ref)
grid on;
hold on
plot(x_kalman)
axis([-Inf Inf -0.3 0.3])
xlabel('t [ms]')
ylabel('x [m]')
title('comparison position x')
hold off
subplot(4,1,2)
plot(y_ref)
grid on;
hold on
plot(y_kalman)
axis([-Inf Inf -0.3 0.3])
xlabel('t [ms]')
ylabel('y [m]')
title('comparison position y')
hold off
subplot(4,1,3)
plot(theta_ref)
grid on;
hold on
plot(theta_kalman)
axis([-Inf Inf -6 1])
xlabel('t [ms]')
ylabel('\theta [rad]')
title('comparison angle \theta')
hold off
subplot(4,1,4)
plot(meas_valid)
grid on;
xlabel('t [ms]')
ylabel('meas valid [-]')
title('measuremtens valid')
%axis([-Inf Inf -Inf Inf])

figure('name', 'trajectory experiment 1')
plot(x_ref, y_ref)
grid on;
hold on
plot(x_kalman, y_kalman)
hold off
legend('reference', 'estimated')
xlabel('x [m]')
ylabel('y [m]')
title('position of center cart')



% ============================== Experiment 2 ===============================

rec = readlog('log_gpio_test_film1.xml');

t_input = rec.getData('time');
vA = rec.getData('wheel_speedA');
vB = rec.getData('wheel_speedB');
x_ref = rec.getData('x_ref');
y_ref = rec.getData('y_ref');
theta_ref = rec.getData('theta_ref');
x_kalman = rec.getData('x_kalman');
y_kalman = rec.getData('y_kalman');
theta_kalman = rec.getData('theta_kalman');
meas_valid = rec.getData('meas_valid');

figure('name', 'comparison states experiment 2')
subplot(4,1,1)
plot(x_ref)
grid on
hold on
plot(x_kalman)
axis([-Inf Inf -0.3 0.3])
hold off
xlabel('t [ms]')
ylabel('x [m]')
title('comparison position x')
subplot(4,1,2)
plot(y_ref)
grid on
hold on
plot(y_kalman)
axis([-Inf Inf -0.3 0.3])
hold off
xlabel('t [ms]')
ylabel('y [m]')
title('comparison position y')
subplot(4,1,3)
plot(theta_ref)
grid on
hold on
plot(theta_kalman)
axis([-Inf Inf -6 1])
hold off
xlabel('t [ms]')
ylabel('\angle [rad]')
title('comparison angle \theta')
subplot(4,1,4)
plot(meas_valid)
grid on
xlabel('t [ms]')
ylabel('meas valid [-]')
title('measurements valid')
%axis([-Inf Inf -Inf Inf])

figure('name', 'errors in tracking the trajectory in experiment 2')
subplot(4,1,1)
plot(x_ref - x_kalman)
ylabel('x_{ref} - x_{est} [m]')
grid on
subplot(4,1,2)
plot(y_ref - y_kalman)
ylabel('y_{ref} - y_{est} [m]')
grid on
subplot(4,1,3)
plot(theta_ref - theta_kalman)
ylabel('\theta_{ref} - \theta_{est} [rad]')
grid on
subplot(4,1,4)
plot(meas_valid)
ylabel({'measurement','valid'})
grid on

figure('name', 'trajectory experiment 2')
plot(x_ref, y_ref)
grid on
hold on
plot(x_kalman, y_kalman)
hold off
legend('reference', 'estimated')
xlabel('x [m]')
ylabel('y [m]')
title('position of center cart')