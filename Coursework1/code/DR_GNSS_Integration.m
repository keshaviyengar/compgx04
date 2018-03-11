function [ned_positions,ned_velocities] = DR_GNSS_Integration(time,gnss_positions,...
    gnss_vel,dr_positions,dr_vel)

%COMPGX04: Robot Vision and Navigation
%Keshav Iyengar
%task 2: Car DR/GNSS Integration

%define constants
Define_Constants

%initialize corrected dr variables
dr_positions_new = gnss_positions(1:2,1);
dr_vel_new = gnss_vel(1:2,1);

%initialize state vector to zeros
x_k_minus_1 = [0;0;0;0];

%initalize state estimation covariance matrix
sigma_v = 0.05;
sigma_r = 10;
[R_N,R_E] = Radii_of_curvature(gnss_positions(1,1));
h0 = gnss_positions(3,1);
P_k_minus_1 = diag([sigma_v^2 sigma_v^2 sigma_r^2/(R_N + h0)^2 sigma_r^2/((R_E + h0)^2*cos(gnss_positions(1,1))^2)]);

%follow ten-step kalman filter
for epoch=2:size(time,1)
    [R_N,R_E] = Radii_of_curvature(gnss_positions(1,epoch));
    %compute transition matrix
    tau_s = 0.5; %propogation interval
    phi_k_minus_1 = diag(ones(1,4));
    phi_k_minus_1(3,1) = tau_s / (R_N + gnss_positions(3,epoch-1));
    phi_k_minus_1(4,2) = tau_s / ((R_E + gnss_positions(3,epoch-1))*cos(gnss_positions(1,epoch-1)));
    
    %compute system noise matrix
    s_dr = 0.01;
    Q_k_minus_1 = zeros(4,4);
    Q_k_minus_1(1,1) = s_dr*tau_s;
    Q_k_minus_1(3,1) = 1/2 * (s_dr*tau_s^2) / (R_N + gnss_positions(3,epoch-1));
    Q_k_minus_1(2,2) = Q_k_minus_1(1,1);
    Q_k_minus_1(4,2) = 1/2 * (s_dr*tau_s^2) / ((R_E + gnss_positions(3,epoch-1))*cos(gnss_positions(1,epoch-1)));
    Q_k_minus_1(1,3) = Q_k_minus_1(3,1);
    Q_k_minus_1(3,3) = 1/3 * (s_dr*tau_s^3) / (R_N + gnss_positions(3,epoch-1))^2;
    Q_k_minus_1(2,4) = Q_k_minus_1(4,2);
    Q_k_minus_1(4,4) = 1/3 * (s_dr*tau_s^3) / ((R_E + gnss_positions(3,epoch-1))^2*cos(gnss_positions(1,epoch-1))^2);
    
    %propogate state estimates
    x_k = phi_k_minus_1*x_k_minus_1;
    %propogate error covariance matrix
    P_k = phi_k_minus_1*P_k_minus_1*phi_k_minus_1' + Q_k_minus_1;
    
    %compute measurement matrix
    H_k = ...
        [0 0 -1 0;
        0 0 0 -1;
        -1 0 0 0;
        0 -1 0 0];
    
    %compute measurement noise covariance matrix
    sigma_gr = 10;
    sigma_gv = 0.05;
    R_k = diag([sigma_gr^2/(R_N + gnss_positions(3,epoch))^2 ...
        sigma_gr^2/((R_E + gnss_positions(3,epoch))^2*cos(gnss_positions(1,epoch))^2) ...
        sigma_gv^2 ...
        sigma_gv^2]);
    
    %compute kalman gain
    K_k = P_k*H_k'/(H_k*P_k*H_k' + R_k);
    
    %formulate the measurement innovation vector
    d_z = [gnss_positions(1:2,epoch) - dr_positions(:,epoch);
        gnss_vel(1:2,epoch) - dr_vel(:,epoch)] - H_k*x_k;
    
    %update state estimates
    x_k_plus = x_k + K_k*d_z;
    
    %update error covariance matrix
    P_k_plus = (eye(4,4) - K_k*H_k)*P_k;
    
    %correct dead reackoning
    dr_positions_new(:,epoch) = dr_positions(:,epoch) - x_k_plus(3:4);
    dr_vel_new(:,epoch) = dr_vel(:,epoch) - x_k_plus(1:2);
    
    %update variables
    x_k_minus_1 = x_k_plus;
    P_k_minus_1 = P_k_plus;
end
%append height to dr positions

ned_positions = dr_positions_new;
ned_velocities = dr_vel_new; 
end