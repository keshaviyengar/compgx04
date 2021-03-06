%COMPGX04: Robot Vision and Navigation
%Keshav Iyengar

%this function computes the meridan and transverse radius
%inputs: [L_k_-_1,lamda_k_-_1], [V_n_k, V_e_k], t_k, t_k_-_1, height
%outputs: L_k,lamda_k

function [Lat,Long] = Vel2LatAndLong(last_lat_long,avg_vel,t_k,t_k_minus_1,h)
[R_N,R_E] = Radii_of_curvature(last_lat_long(1,1));
Lat = last_lat_long(1,1) + avg_vel(1,1)*(t_k - t_k_minus_1) / (R_N + h);
Long = last_lat_long(2,1) + avg_vel(2,1)*(t_k - t_k_minus_1) / ((R_E + h)*cos(Lat));
end