function heading_solutions = GyroOnlyKalmanFilter(time,heading,gyro_heading)
%COMPGX04: Robot Vision and Navigation
%Keshav Iyengar

%define constants
Define_Constants

%initalize matrices
solutions = zeros(1,size(time,1));

%initalize kalman filter state vector
x_k_minus_1 = [0;0];
sigma_bias = 1;
sigma_heading = 4*deg_to_rad;
P_k_minus_1 = [sigma_heading^2 0;0 sigma_bias^2];
%compute transition matrix
tau_s = 0.5;
phi_k_minus_1 = [1 tau_s;0 1];

% Kalman filter
%compute system noise covariance matrix
S_rg = e-4;%Gyro random noise with power spectral density (PSD)
S_bgd = 3e-6;%Gyro bias variation with PSD 
Q_k_minus_1 = ...
    [S_rg*tau_s+1/3*S_bgd*tau_s^3 1/2*S_bgd*tau_s^2;
    1/2*S_bgd*tau_s^2 S_bgd*tau_s];

for epoch=1:size(time,1)
    %use transition matrix to propogate state estimate
    x_k = phi_k_minus_1*x_k_minus_1;
    
    %propogate state covariance matrix
    P_k = phi_k_minus_1*P_k_minus_1*phi_k_minus_1' + Q_k_minus_1;
    
    %compute measurement matrix
    H_k = [-1 0];
    
    %measurement innovation vector
    d_z = heading(epoch) - gyro_heading(epoch) - H_k*x_k;
    
    %compute measurement noise covaraince matrix
    sigma_m = 4*deg_to_rad;
    R_k = diag([sigma_m^2]);
    
    %Compute Kalman Gain matrix
    K_k = P_k*H_k'/(H_k*P_k*H_k' + R_k);
    
    %update state estimates
    x_k_plus = x_k + K_k*d_z;
    P_k_plus = (eye(size(P_k,1)) - K_k*H_k)*P_k;
    
    %append solutions
    solutions(:,epoch) = gyro_heading(epoch) - x_k_plus(1);
    
    %update the k-1 variables
    x_k_minus_1 = x_k_plus;
    P_k_minus_1 = P_k_plus;
end
heading_solutions = solutions;
end
