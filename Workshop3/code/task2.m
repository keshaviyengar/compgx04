%COMPGX04: Robot Vision and Navigation
%Keshav Iyengar
%task 2: Car DR/GNSS Integration
clear variables

%define constants
Define_Constants

%extract data from given files
load('dr_vel.mat','dr_vel');
load('dr_positions.mat','dr_positions');
M = csvread('Workshop3_GNSS_Pos_Vel_NED.csv');
time = M(:,1);
gnss_positions = [deg_to_rad*M(:,2) deg_to_rad*M(:,3) M(:,4)]';
gnss_vel = M(:,5:7)';

%initialize corrected dr variables
dr_positions_new = gnss_positions(1:2,1);
dr_vel_new = gnss_vel(1:2,1);

%initialize state vector to zeros
x0 = [0;0;0;0];
x_k_minus_1 = x0;

%initalize state estimation covariance matrix
sigma_v = 0.1;
sigma_r = 10;
[R_N,R_E] = Radii_of_curvature(gnss_positions(1,1));
h0 = gnss_positions(3,1);
P0 = diag([sigma_v^2 sigma_v^2 sigma_r^2/(R_N + h0)^2 sigma_r^2/((R_E + h0)^2*cos(gnss_positions(1,1))^2)]);

P_k_minus_1 = P0;

%follow ten-step kalman filter
for epoch=2:size(time,1)
    [R_N,R_E] = Radii_of_curvature(gnss_positions(1,epoch));
    %compute transition matrix
    tau_s = 0.5; %propogation interval
    phi_k_minus_1 = diag(ones(1,4));
    phi_k_minus_1(3,1) = tau_s / (R_N + gnss_positions(3,epoch-1));
    phi_k_minus_1(4,2) = tau_s / ((R_E + gnss_positions(3,epoch-1))*cos(gnss_positions(1,epoch-1)));
    
    %compute system noise matrix
    s_dr = 0.2;
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
    sigma_gr = 5;
    sigma_gv = 0.02;
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
dr_positions_new = [dr_positions_new;gnss_positions(3,:)];
dr_vel_new = [dr_vel_new;gnss_vel(3,:)];

disp('Positions: ')
disp(dr_positions_new(1:2,:)*rad_to_deg)
% disp('Velocities: ')
% disp(dr_vel_new)