function [x_est,P_matrix] = Initialise_GNSS_KF(r_eb_e,v_eb_e,d_rho_c,dd_rho_c)
%Initialise_GNSS_KF - Initializes the GNSS EKF state estimates and error
%covariance matrix
%
% This function created 30/11/2016 by Paul Groves
% edited 11/02/2018 by Keshav Iyengar
%
% Outputs:
%   x_est                 Kalman filter estimates:
%     Rows 1-3            estimated ECEF user position (m)
%     Rows 4-6            estimated ECEF user velocity (m/s)
%     Row 7               estimated receiver clock offset (m) 
%     Row 8               estimated receiver clock drift (m/s)
%   P_matrix              state estimation error covariance matrix

% Copyright 2016, Paul Groves
% License: BSD; see license.txt for details

% Begins

% Initialise state estimates
x_est = [r_eb_e;v_eb_e;d_rho_c;dd_rho_c];

% Initialise error covariance matrix
P_matrix =  zeros(8);
P_matrix(1,1) = 10^2; %position
P_matrix(2,2) = 10^2;
P_matrix(3,3) = 10^2;
P_matrix(4,4) = 0.05^2; %velocity
P_matrix(5,5) = 0.05^2;
P_matrix(6,6) = 0.05^2;
P_matrix(7,7) = 100000; %clock offset std deviation
P_matrix(8,8) = 200; %clock drift std deviation

% Ends