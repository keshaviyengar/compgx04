function [initial_pos] = SingleEpochPosNoInit(time,sat_id,pseudo_ranges)
%COMPGX04: Robot Vision and Navigation
%Keshav Iyengar
%Task 1b: Mobile GNSS Positioning using Least-Squares Estimation

%define constants
Define_Constants

%b. cartesian ecef positions of satellites at time 0
sat_r_es_e_all = zeros(3,size(sat_id,2));
sat_v_es_e_all = zeros(3,size(sat_id,2));

epoch = 1;
for j=1:size(sat_id,2)
    %get value for satellite
    [sat_r_es_e,sat_v_es_e] = Satellite_position_and_velocity(time(epoch),sat_id(j));
    %append to matrix for all satellites (possible error here)
    sat_r_es_e_all(:,j) = sat_r_es_e';
    sat_v_es_e_all(:,j) = sat_v_es_e';
end

%set initial guess to current solution
r_eb_e = [0;0;0];

d_rho_c = 0;
prev_r_eb_e = [0;0;0]; %previous solution
tol = 0.10; %10cm tolerence
%set starting error to inf
error = inf;
while (error > tol)
    %c. Predict range from the approximate user position
    r_a_all = zeros(1,size(sat_id,2));
    for j=1:size(sat_id,2)
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
    end
    
    %d. compute line-of-sight unit vector for satellite
    u_a_all = zeros(3,size(sat_id,2));
    for j=1:size(sat_id,2)
        u_a = (sat_r_es_e_all(:,j) - r_eb_e) / r_a_all(j);
        u_a_all(:,j) = u_a;
    end
    
    %e. Formulate the predicted state vector, measurement innovation vector and
    %measurement matrix
    %predicted state vector
    state_x = [r_eb_e;d_rho_c];
    
    %measurement innovation vector
    d_z = zeros(size(sat_id,2),1);
    for j=1:size(sat_id,2)
        d_z(j,1) = pseudo_ranges(epoch,j) - r_a_all(1,j) - d_rho_c;
    end
    
    %measurement matrix
    H = zeros(size(sat_id,2),4);
    for j=1:size(sat_id,2)
        H(j,:) = [-u_a_all(:,j)' 1];
    end
    
    %f. Compute position and reciever clock offset using unweighted
    %least-squares
    state_x_new = state_x + inv((H'*H))*H'*d_z;
    %set solution to current solution
    r_eb_e = state_x_new(1:3,1);
    d_rho_c = state_x_new(4,1);
    error = abs(norm(r_eb_e) - norm(prev_r_eb_e));
    prev_r_eb_e = r_eb_e;
end
%g. Convert cartesian ecef position to latitude, longitude and height
[x_L_b,x_lambda_b,x_h_b,~] = pv_ECEF_to_NED(r_eb_e,d_rho_c);
initial_pos = [x_L_b;x_lambda_b;x_h_b];
end