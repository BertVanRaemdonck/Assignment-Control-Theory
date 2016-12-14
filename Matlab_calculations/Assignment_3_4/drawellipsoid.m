function [ contour ] = drawellipsoid( P )
%DRAWELLIPSOID Returns contour of a confidence ellipsoid, given a covariance
%matrix P
%   P is a 2x2 matrix representing the convariance matrix of the estimation
%   error. contour is 1000x2 matrix representing the x and y coordinate of
%   the contour points. Plot the contour as follows: 
%        plot(contour(:,1), contour(:,2))

theta = linspace(0,2*pi,1e3)';
[U,D] = eig(P);
z1 = sqrt(D(1,1))*cos(theta);
z2 = sqrt(D(2,2))*sin(theta);
contour = 0.1*(U * [z1(:)'; z2(:)'])';

end

