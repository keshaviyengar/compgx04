function [dr_ned_positions,dr_ned_velocities] = DeadReckoningOnly(time,forward_speed,heading,r_eb_n)
%COMPGX04: Robot Vision and Navigation
%Keshav Iyengar
%task 1: Car Dead Reckoning

%define constants
Define_Constants

dr_positions = zeros(2,size(time,1));
dr_positions(:,1) = r_eb_n(1:2);

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
   h = r_eb_n(3);
   [L_k,lambda_k] = Vel2LatAndLong(dr_positions(:,epoch-1),avg_vel(:,epoch),time(epoch),time(epoch-1),h);
   dr_positions(1,epoch) = L_k;
   dr_positions(2,epoch) = lambda_k;
   %compute instantenous DR velocity at each epoch
   dr_vel(1,epoch) = 2*avg_vel(1,epoch) - dr_vel(1,epoch-1);
   dr_vel(2,epoch) = 2*avg_vel(2,epoch) - dr_vel(2,epoch-1);
end
dr_ned_positions = dr_positions;
dr_ned_velocities = dr_vel;
end