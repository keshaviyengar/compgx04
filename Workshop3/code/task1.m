%COMPGX04: Robot Vision and Navigation
%Keshav Iyengar
%task 1: Car Dead Reckoning
clear variables

%define constants
Define_Constants

%extract data from given files
M = csvread('Workshop3_Speed_Heading.csv');
time = M(:,1);
forward_speed = M(:,2);
heading = M(:,3)*deg_to_rad;

dr_positions = zeros(2,size(time,1));
dr_positions(:,1) = [50.4249580;-3.5957974]*deg_to_rad;

avg_vel = zeros(2,size(time,1)-1);
dr_vel = zeros(2,size(time,1)-1);
dr_vel(1,1) = forward_speed(1)*cos(heading(1));
dr_vel(2,1) = forward_speed(1)*sin(heading(1));

for epoch=2:size(time,1)
   %compute average velocity
   avg_vel(:,epoch) = ...
       1/2*[cos(heading(epoch)) + cos(heading(epoch-1));
       sin(heading(epoch)) + sin(heading(epoch-1))] * forward_speed(epoch);
   
   %compute latitiude and longitude position
   h = 37.4;
   [L_k,lambda_k] = Vel2LatAndLong(dr_positions(:,epoch-1),avg_vel(:,epoch),time(epoch),time(epoch-1),h);
   dr_positions(1,epoch) = L_k;
   dr_positions(2,epoch) = lambda_k;
   %compute instantenous DR velocity at each epoch
   dr_vel(1,epoch) = 2*avg_vel(1,epoch) - dr_vel(1,epoch-1);
   dr_vel(2,epoch) = 2*avg_vel(2,epoch) - dr_vel(2,epoch-1);
end

disp('Positions: ')
disp(dr_positions*rad_to_deg)
disp('Velocities: ')
disp(dr_vel)