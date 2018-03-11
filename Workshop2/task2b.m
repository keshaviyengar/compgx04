%COMPGX04: Robot Vision and Navigation
%Keshav Iyengar
%task 2b: GNSS Kalman Filter multiple epochs
clear variables

%define constants
Define_Constants

%extract data from given files
M = csvread('Workshop2_Pseudo_ranges.csv');
V = csvread('Workshop2_Pseudo_range_rates.csv');
P = csvread('Workshop2_GNSS_Pos_ECEF.csv');
t = M(2:end,1);
sat_id = M(1,2:end);
pseudo_ranges = M(2:end,2:end);
pseudo_range_rates = V(2:end,2:end);
ecef_positions = P(1:end,2:end);

%initalize matrices
solutions = zeros(size(t,1),6);
sat_r_es_e_all = zeros(3,size(sat_id,2));
sat_v_es_e_all = zeros(3,size(sat_id,2));
u_a_all = zeros(3,size(sat_id,2));
r_a_all = zeros(1,size(sat_id,2));
r_a_dot_all = zeros(1,size(sat_id,2));
d_z = zeros(2*size(sat_id,2),1);

%initalize kalman filter state vector
[x_k_minus_1,P_k_minus_1] = Initialise_GNSS_KF;

%compute transition matrix
prop_interval = 1;
phi_k_minus_1 = ...
    [eye(3,3) prop_interval*eye(3,3) zeros(3,1) zeros(3,1);
    zeros(3,3) eye(3,3) zeros(3,1) zeros(3,1);
    zeros(1,3) zeros(1,3) 1 prop_interval;
    zeros(1,3) zeros(1,3) 0 1];

% Kalman filter
%compute system noise covariance matrix
S_a = 5;
S_c_phi = 0.01;
S_c_f = 0.04;
Q_k_minus_1 = ...
    [1/3*S_a*prop_interval^3*eye(3,3) 1/2*S_a*prop_interval^2*eye(3,3) zeros(3,1) zeros(3,1);
    1/2*S_a*prop_interval^2*eye(3,3) S_a*prop_interval*eye(3,3) zeros(3,1) zeros(3,1);
    zeros(1,3) zeros(1,3) S_c_phi*prop_interval + 1/3*S_c_f*prop_interval^3 1/2*S_c_f*prop_interval^2;
    zeros(1,3) zeros(1,3) 1/2*S_c_f*prop_interval^2 S_c_f*prop_interval];

for epoch=1:size(t,1)
    %use transition matrix to propogate state estimate
    x_k = phi_k_minus_1*x_k_minus_1;
    
    %propogate state covariance matrix
    P_k = phi_k_minus_1*P_k_minus_1*phi_k_minus_1' + Q_k_minus_1;
    
    %compute line of sight vectors
    for j=1:size(sat_id,2)
        r_eb_e = x_k(1:3,1);
        v_eb_e = x_k(4:6,1);
        %get value for satellite
        [sat_r_es_e,sat_v_es_e] = Satellite_position_and_velocity(t(epoch),sat_id(j));
        %append to matrix for all satellites (possible error here)
        sat_r_es_e_all(:,j) = sat_r_es_e';
        sat_v_es_e_all(:,j) = sat_v_es_e';
        
        %c. Predict range from the approximate user position
        %initial range computation
        r_a = sqrt((eye(3,3)*sat_r_es_e_all(:,j) - r_eb_e)' * ...
            (eye(3,3)*sat_r_es_e_all(:,j) - r_eb_e));
        %Sagnac effect compensation matrix
        C_e = [1               omega_ie*r_a/c 0;
            -omega_ie*r_a/c 1              0;
            0               0              1];
        %recalcuate range with Sagnac effect compensation
        r_a = sqrt((C_e*sat_r_es_e_all(:,j) - r_eb_e)' * ...
            (C_e*sat_r_es_e_all(:,j) - r_eb_e));
        r_a_all(:,j) = r_a;
        
        %compute line of sight vector
        u_a = (sat_r_es_e_all(:,j) - r_eb_e) / r_a_all(:,j);
        u_a_all(:,j) = u_a;
        
        %calculate range rates for each satellite
        r_a_dot = u_a'*(C_e*(sat_v_es_e_all(:,j) + Omega_ie*sat_r_es_e_all(:,j)) - (v_eb_e + Omega_ie*r_eb_e));
        r_a_dot_all(:,j) = r_a_dot;
        
        %formulate measurement innovation vector
        d_z(j,1) = pseudo_ranges(epoch,j) - r_a_all(1,j) - x_k(7,1);
        d_z(j+size(sat_id,2),1) = pseudo_range_rates(epoch,j) - r_a_dot_all(1,j) - x_k(8,1);
    end
    
    %compute measurement matrix
    H_k = zeros(2*size(sat_id,2),2*size(u_a_all,1)+2);
    for k=1:size(u_a_all,2)
        H_k(k,:) = [-u_a_all(:,k)' zeros(1,3) 1 0];
        H_k(k+size(sat_id,2),:) = [zeros(1,3) -u_a_all(:,k)' 0 1];
    end
    
    %compute measurement noise covaraince matrix
    var_p(1,1:size(pseudo_ranges,2)) = 10^2;
    var_r(1,1:size(pseudo_range_rates,2)) = 0.05^2;
    R_k = diag([var_p var_r]);
    
    %Compute Kalman Gain matrix
    K_k = P_k*H_k'/(H_k*P_k*H_k' + R_k);
    
    %update state estimates
    x_k_plus = x_k + K_k*d_z;
    P_k_plus = (eye(size(P_k,1)) - K_k*H_k)*P_k;
    
    %append solutions
    [L_b,lambda_b,h_b,v_eb_n] = pv_ECEF_to_NED(x_k_plus(1:3),x_k_plus(4:6));
    solutions(epoch,:) = [rad2deg(L_b),rad2deg(lambda_b),h_b,v_eb_n'];
    
    %update the k-1 variables
    x_k_minus_1 = x_k_plus;
    P_k_minus_1 = P_k_plus;
end

disp('positions: ')
num2str(solutions(:,1:3),'%.2f')
disp('velocities: ')
num2str(solutions(:,4:6),'%.2f')
