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
assignment_3_2_2


%% Defining system of position sensor fusion
close all
clc

%Ts = 1/100;

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


clear show_figures6
