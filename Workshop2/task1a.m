%COMPGX04: Robot Vision and Navigation
%Keshav Iyengar
%task 1a: Basic Kalman Filter first epoch
clear all

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

%initalise state vector
x0 = [2447019;-5884199;-284783;184;77;0];
P0 = diag([100 100 100 25 25 25]);

epoch = 1;

x_k_minus_1 = x0;
P_k_minus_1 = P0;
%compute transition matrix
prop_interval = 1;
phi_k_minus_1 = [eye(3,3) prop_interval*eye(3,3); zeros(3,3) eye(3,3)];

%compute system noise covariance matrix
S = 5; %power spectral density
Q_k_minus_1 = [1/3*S*prop_interval^3*eye(3,3) 1/2*S*prop_interval^2*eye(3,3);
     1/2*S*prop_interval^2*eye(3,3) S*prop_interval*eye(3,3)];
 
%propogate state estimates
x_k = phi_k_minus_1*x_k_minus_1;
P_k = phi_k_minus_1*P_k_minus_1*phi_k_minus_1' + Q_k_minus_1;

%measurement matrix
H = eye(3,6);

%compute measurement noise covariance matrix
sigma_r = 2.5;
R_k = diag([sigma_r^2 sigma_r^2 sigma_r^2]);

%compute kalman gain
K_k = P_k*H'/(H*P_k*H' + R_k);

%measurement innovation matrix d_z
d_z = ecef_positions(epoch,:)' - x_k(1:3);

%update state estimates
x_k_new = x_k + K_k*d_z;

%update error covariance
P_k_new = (eye(6,6)-K_k*H)*P_k;

%display solution
[L_b,lambda_b,h_b,v_eb_n] = pv_ECEF_to_NED(x_k_new(1:3),x_k_new(4:end));
disp('Position: ')
disp(num2str([rad2deg(L_b) rad2deg(lambda_b) h_b]','%.5f'))
disp('Velocity')
disp(v_eb_n)

