%COMPGX04: Robot Vision and Navigation
%Keshav Iyengar
%Workshop 2: Multi-epoch Positioning

%bugs: height is incorrect

%This function computes the least-square position solution in NED given:
%time: current time interval
%M matrix containing th pseudo_range information
%inital_guess_pos, inital_guess_vel
%d_rho_c: clock offset
%Returns
%r_ned_eb_e: Solution, latitude, longitude and ehight
%d_rho_c: Predicted clock offsets
function [r_ned_eb_e,d_rho_c] = single_epoch_positioning(t,M,...,
    init_guess_pos,init_guess_vel,d_rho_c)
%define constants
Define_Constants

%extract data from Workshop1_Pseudo_ranges.csv
sat_id = M(1,2:end);
pseudo_ranges = M(2:end,2:end);

%b. cartesian ecef positions of satellites at time 0
sat_r_es_e_all = zeros(3,size(sat_id,2));
sat_v_es_e_all = zeros(3,size(sat_id,2));
for j=1:size(sat_id,2)
    %get value for satellite
    [sat_r_es_e,sat_v_es_e] = Satellite_position_and_velocity(t,sat_id(j));
    %append to matrix for all satellites (possible error here)
    sat_r_es_e_all(:,j) = sat_r_es_e';
    sat_v_es_e_all(:,j) = sat_v_es_e';
end

%set initial guess to current solution
r_eb_e = init_guess_pos;
v_eb_e = init_guess_vel;
[x_L_b,x_lambda_b,x_h_b,~] = pv_ECEF_to_NED(r_eb_e,d_rho_c);
ned_eb_e = [x_L_b;x_lambda_b;x_h_b];
tol = 0.10; %10cm tolerence
%set starting error to inf
error = inf;
while (error > tol)
    %c. Predict range from the approximate user position
    disp(r_eb_e);
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
        d_z(j,1) = pseudo_ranges(1,j) - r_a_all(1,j) - d_rho_c;
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
    
    %calculate error in ned
    %g. Convert cartesian ecef position to latitude, longitude and height
    [x_L_b,x_lambda_b,x_h_b,~] = pv_ECEF_to_NED(r_eb_e,d_rho_c);
    %set last solution to old
    ned_eb_e_old = ned_eb_e;
    %set new solution
    ned_eb_e = [x_L_b;x_lambda_b;x_h_b];
    %compute error
    error = abs(norm(ned_eb_e) - norm(ned_eb_e_old));
    disp('error: ');
    disp(error);
end

%solution
disp('solution found: ');
disp('Latitude: ');
disp(rad2deg(ned_eb_e(1,1)));
disp('Longitude: ');
disp(rad2deg(ned_eb_e(2,1)));
disp('Height: ')
disp(ned_eb_e(3,1));
end