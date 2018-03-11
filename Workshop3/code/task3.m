%COMPGX04: Robot Vision and Navigation
%Keshav Iyengar
%task 3: UAV INS/GNSS Integration
clear variables

%define constants
Define_Constants

%extract data from given files
load('dr_vel.mat','dr_vel');
M = csvread('Aircraft_inertial.csv');
time = M(:,1);
inertial_L_b = deg_to_rad*M(:,2)';
interial_lambda_b = deg_to_rad*M(:,3)';
interial_h_b = M(:,4)';
inertial_veb_n = M(:,5:7)';
inertial_angles = deg_to_rad*M(:,8:10)';
f_ib_b = M(:,11:13)';

inertial_r_eb_e = zeros(3,size(time,1));
inertial_v_eb_e = zeros(3,size(time,1));
%gnss solutions from task 2 (fix when you get task 2 working)
load('dr_positions_new.mat','dr_positions_new');
load('dr_vel_new.mat','dr_vel_new');
gnss_r_eb_e = zeros(3,size(time,1));
gnss_v_eb_e = zeros(3,size(time,1));
for t=1:size(time,1)
    [r_eb_e,v_eb_e] = pv_NED_to_ECEF(dr_positions_new(1,t),dr_positions_new(2,t),dr_positions_new(3,t),dr_vel_new(:,t));
    gnss_r_eb_e(:,t) = r_eb_e;
    gnss_v_eb_e(:,t) = v_eb_e;
end

inertial_C_n_b = zeros(3,3,size(time,1));
inertial_C_b_e = zeros(3,3,size(time,1));
%get the transforms of euler angles and ecef velocities and positions
% ned transform for each epoch
for t=1:size(time,1)
    %get transform from euler angles
    inertial_C_n_b(:,:,t) = Euler_to_CTM(inertial_angles(:,t))';
    %get the ecef representation
    [r_eb_e,v_eb_e,C_b_e] = NED_to_ECEF(inertial_L_b(t),...
        interial_lambda_b(t),interial_h_b(t),inertial_veb_n(:,t),...
        inertial_C_n_b(:,:,t));
    %set to storage variables
    inertial_r_eb_e(:,t) = r_eb_e;
    inertial_v_eb_e(:,t) = v_eb_e;
    inertial_C_b_e(:,:,t) = C_b_e;
    
end

%initalize state vector and error covariance matrix
[x_k_minus_1,P_k_minus_1] = Initialise_Integration_KF;

inertial_r_eb_e_new = zeros(3,size(time,1));
inertial_v_eb_e_new = zeros(3,size(time,1));
inertial_C_b_e_new = zeros(3,3,size(time,1));
%follow the ten-step kalman filter
for epoch=1:size(time,1)
    %compute transition matrix
    tau_s = 0.5;
    F21 = Calculate_F21(inertial_C_b_e(:,:,epoch),f_ib_b(:,epoch));
    F23 = Calculate_F23(inertial_r_eb_e(:,epoch),inertial_L_b(:,epoch));
    phi_k_minus_1 = [...
        (eye(3)-Omega_ie)*tau_s zeros(3) zeros(3) zeros(3) inertial_C_b_e(:,:,epoch)*tau_s;...
        F21*tau_s (eye(3)-2*Omega_ie)*tau_s F23*tau_s inertial_C_b_e(:,:,epoch)*tau_s zeros(3);...
        zeros(3) eye(3)*tau_s eye(3) zeros(3) zeros(3);...
        zeros(3) zeros(3) zeros(3) eye(3) zeros(3);...
        zeros(3) zeros(3) zeros(3) zeros(3) eye(3)];
    
    %compute system noise covariance matrix
    S_rg = LC_KF_config.gyro_noise_PSD;
    S_ra = LC_KF_config.accel_noise_PSD;
    S_bad = LC_KF_config.accel_bias_PSD;
    S_bgd = LC_KF_config.gyro_bias_PSD;
    Q_k_minus_1 = [...
        S_rg*tau_s*eye(3) zeros(3) zeros(3) zeros(3) zeros(3);...
        zeros(3) S_ra*tau_s*eye(3) zeros(3) zeros(3) zeros(3);...
        zeros(3) zeros(3) zeros(3) zeros(3) zeros(3);...
        zeros(3) zeros(3) zeros(3) S_bad*tau_s*eye(3) zeros(3);...
        zeros(3) zeros(3) zeros(3) zeros(3) S_bgd*tau_s*eye(3)];
    
    %propogate state estimates
    x_k = phi_k_minus_1*x_k_minus_1;
    %propogate error covariance matrix
    P_k = phi_k_minus_1*P_k_minus_1*phi_k_minus_1' + Q_k_minus_1;
    
    %Measurement Matrix
    H_k = [zeros(3) zeros(3) -eye(3) zeros(3) zeros(3);
        zeros(3) -eye(3) zeros(3) zeros(3) zeros(3)];
    
    %measurement noise covariance matrix
    sigma_r = LC_KF_config.pos_meas_SD;
    sigma_v = LC_KF_config.vel_meas_SD ;
    R_k = [sigma_r^2*eye(3) zeros(3);
        zeros(3) sigma_v^2*eye(3)];
    
    %compute kalman gain
    K_k = P_k*H_k'/(H_k*P_k*H_k' + R_k);
    
    %measurement innovation vector
    d_z = [gnss_r_eb_e(:,epoch) - inertial_r_eb_e(:,epoch) + x_k(7:9);
        gnss_v_eb_e(:,epoch) - inertial_v_eb_e(:,epoch) + x_k(4:6)];
    
    %update state estimates
    x_k_plus = x_k + K_k*d_z;
    %update error covariance matrix
    P_k_plus = (eye(size(P_k)) - K_k*H_k)*P_k;
    
    %correct inertial values
    inertial_C_b_e_new(:,:,epoch) = (eye(3) - Skew_symmetric(x_k_plus(1:3)))*inertial_C_b_e(:,:,epoch);
    inertial_r_eb_e_new(:,epoch) = inertial_r_eb_e(:,epoch) - x_k_plus(7:9);
    inertial_v_eb_e_new(:,epoch) = inertial_v_eb_e(:,epoch) - x_k_plus(4:6);
    
    %update variables
    x_k_minus_1 = x_k_plus;
    P_k_minus_1 = P_k_plus;
end

%convert new corrections to correct units
sol_ned_pos = zeros(3,size(time,1));
sol_ned_vel = zeros(3,size(time,1));
sol_euler = zeros(3,size(time,1));
for t=1:size(time,1)
    [L_b,lambda_b,h_b,v_eb_n,C_b_n] = ECEF_to_NED(inertial_r_eb_e_new(:,t),inertial_v_eb_e_new(:,t),inertial_C_b_e_new(:,:,t));
    sol_ned_pos(:,t) = [L_b*rad_to_deg;lambda_b*rad_to_deg;h_b];
    sol_ned_vel(:,t) = v_eb_n;
    sol_euler(:,t) = rad_to_deg*CTM_to_Euler(C_b_n) ;
end
disp('position: ')
disp(sol_ned_pos(1:2,:))
% disp('height: ')
% disp(sol_ned_pos(3,:)');

