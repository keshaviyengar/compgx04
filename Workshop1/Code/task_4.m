%COMPGX04: Robot Vision and Navigation
%Keshav Iyengar
%Task 4: Velocity Determination
clear variables

%bugs v_down seems to be off, might be due to satelitte 18 (outlier)

%Task 1a: Single-epoch Positioning with Initialisation
%define constants
Define_Constants

%extract data from Workshop1_Pseudo_ranges.csv
M = csvread('Workshop1_Pseudo_ranges.csv');
V = csvread('Workshop1_Pseudo_range_rates.csv');
t = M(2:end,1);
sat_id = M(1,2:end);
pseudo_ranges = M(2:end,2:end);
pseudo_range_rates = V(2:end,2:end);

%a. convert latitude, longititude and height to cartesian ECEF position
L_b = deg2rad(-33.821075); %latitude (rad)
lambda_b = deg2rad(151.188496); %longitude (rad)
h_b = 120; %height (m)
v_eb_n = [0;0;0]; %velocity of body frame w.r.t. ECEF frame, resolved along
% north, east, and down (m/s) 3x1 column vector

[r_eb_e,v_eb_e] = pv_NED_to_ECEF(L_b,lambda_b,h_b,v_eb_n);
%   r_eb_e        Cartesian position of body frame w.r.t. ECEF frame, resolved
%                 along ECEF-frame axes (m) 3x1 column vectors
%   v_eb_e        velocity of body frame w.r.t. ECEF frame, resolved along
%                 ECEF-frame axes (m/s) 3x1 column vector

%inital clock offset estimation
d_rho_c = 0;
dd_rho_c = 0;

%skew symmetric matrix
OMEGA_ie = [0 -omega_ie 0;omega_ie 0 0;0 0 0];

%pre-allocate solutions matrices
pos_solutions = zeros(size(t,1),3);
vel_solutions = zeros(size(t,1),3);

for epoch=1:size(t,1)
    %b. cartesian ecef positions of satellites at time 0
    sat_r_es_e_all = zeros(3,size(sat_id,2));
    sat_v_es_e_all = zeros(3,size(sat_id,2));
    r_a_all = zeros(1,size(sat_id,2));
    u_a_all = zeros(3,size(sat_id,2));
    v_a_all = zeros(1,size(sat_id,2));
    d_z = zeros(size(sat_id,2),1);
    d_v_z = zeros(size(sat_id,2),1);
    H = zeros(size(sat_id,2),4);
    
    for j=1:size(sat_id,2)
        %get value for satellite
        [sat_r_es_e,sat_v_es_e] = Satellite_position_and_velocity(t(epoch),sat_id(j));
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
        
        %d. compute line-of-sight unit vector for satellite
        u_a = (sat_r_es_e_all(:,j) - r_eb_e) / r_a_all(j);
        u_a_all(:,j) = u_a;
        
        %velocity determination
        v_a = u_a'*(C_e*(sat_v_es_e_all(:,j) + OMEGA_ie*sat_r_es_e_all(:,j)) - ...
            (v_eb_e + OMEGA_ie*r_eb_e));
        %predicted range rates
        v_a_all(:,j) = v_a;
        
        % For position
        %e. Formulate the predicted state vector, measurement innovation vector and
        %measurement matrix
        %predicted state vector
        state_x = [r_eb_e;d_rho_c];
        %measurement innovation vector
        d_z(j,1) = pseudo_ranges(epoch,j) - r_a_all(1,j) - d_rho_c;
        
        %measurement matrix
        H(j,:) = [-u_a_all(:,j)' 1];
        
        %Same for velocity determination
        %predicted state vector
        state_v_x = [v_eb_e;dd_rho_c];
        %measurement innovation vector
        d_v_z(j,1) = pseudo_range_rates(epoch,j) - v_a_all(1,j) - dd_rho_c;
        
    end
    %f. Compute position and reciever clock offset using unweighted
    %least-squares
    state_x_new = state_x + (H'*H)\H'*d_z;
    r_eb_e = state_x_new(1:3,1);
    d_rho_c = state_x_new(4,1);
    
    %f. Compute velocity and reciever clock offset using unweighted
    %least-squares
    state_v_x_new = state_v_x + (H'*H)\H'*d_v_z;
    v_eb_e = state_v_x_new(1:3,1);
    dd_rho_c = state_v_x_new(4,1);
    
    %g. Convert cartesian ecef position to latitude, longitude and height
    [x_L_b,x_lambda_b,x_h_b,v_eb_n] = pv_ECEF_to_NED(r_eb_e,v_eb_e);
    pos_solutions(epoch,:) = [rad2deg(x_L_b),rad2deg(x_lambda_b),x_h_b];
    vel_solutions(epoch,:) = v_eb_n;
end

disp('position solutions: ')
disp(pos_solutions)
disp('velocity solutions: ')
disp(vel_solutions)

