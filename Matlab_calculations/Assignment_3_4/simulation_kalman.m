clear all; close all; clc;

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
Q_sim = eye(n_x);
% covariance of simulated measurement noise
R_sim = eye(n_y);
% covariance of modeled process noise
Q_mod = eye(n_x);
% covariance of simulated measurement noise
R_mod = eye(n_y);
% initial state
x0 = zeros(n_x,1);
% initial state estimate
x_est0 = zeros(n_x,1);
% feedback values
kx = 0;
ky = 0;
kt = 0;
% trajectory file
filename = 'TrajectoryKalmanExercise.txt';

% ============================== Simulation ===============================
% read file
data = dlmread(filename, '\t', 1, 0);
t = data(:,1)';
x_ref = data(:,2:4)';  % x [m], y [m], theta [rad]
u_ref = data(:,5:6)';  % v [m/s], omega [rad/s]
Ts = t(2) - t(1);
% create signals
x = zeros(n_x, length(t));
u = zeros(n_u, length(t));
y = zeros(n_y, length(t));
e = zeros(n_x, length(t));
x_est = zeros(n_x, length(t));
P_est = zeros(n_x, n_x, length(t));
nu = zeros(n_y, length(t));
S = zeros(n_y, n_y, length(t));
L = zeros(n_x, n_y, length(t));

% initial values
x(:,1) = x0;
x_est(:,1) = x_est0;

% feedback matrix
K = [kx,  0,  0;
      0, ky, kt];

% simulation
for i = 2:length(t)
    % Plant simulation
    x(:,i) = f(x(:,i-1), u(:,i-1), Ts) + (randn(1,n_x)*chol(Q_sim))';
    y(:,i) = h(x(:,i)) + (randn(1,n_y)*chol(R_sim))';

    % Compute Jacobians
    A = Jf(x_est(:,i-1), u(:,i-1), Ts);
    C = Jh(x_est(:,i-1));

    % Kalman prediction step
    x_est(:,i) = f(x_est(:,i-1), u(:,i-1), Ts);
    P_est(:,:,i) = A*P_est(:,:,i-1)*A' + Q_mod;

    % Kalman correction step
    nu(:,i) = y(:,i) - h(x_est(:,i));
    S(:,:,i) = C*P_est(:,:,i)*C' + R_mod;
    L(:,:,i) = P_est(:,:,i)*C'/S(:,:,i);
    x_est(:,i) = x_est(:,i) + L(:,:,i)*nu(:,i);
    P_est(:,:,i) = (eye(n_x)-L(:,:,i)*C)*P_est(:,:,i);

    % Control law
    e(:,i) = local_error(x_ref(:,i), x_est(:,i));
    u(:,i) = u_ref(:,i) + K*e(:,i);
end


