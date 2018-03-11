function plotter(time,gnss_ned_solution,dr_ned_solution,int_ned_solution)
Define_Constants
%plot lattitude and longitude
figure
plot(rad_to_deg*gnss_ned_solution(1,1)',rad_to_deg*gnss_ned_solution(2,1)','-s','MarkerSize',5,...
    'MarkerEdgeColor','red');
hold on;
p1 = plot(rad_to_deg*gnss_ned_solution(1,:)',rad_to_deg*gnss_ned_solution(2,:),'b');
hold on;
plot(rad_to_deg*dr_ned_solution(1,1)',rad_to_deg*dr_ned_solution(2,1)','-s','MarkerSize',5,...
    'MarkerEdgeColor','red');
hold on;
p2 = plot(rad_to_deg*dr_ned_solution(1,:)',rad_to_deg*dr_ned_solution(2,:)','r');
hold on;
plot(rad_to_deg*int_ned_solution(1,1)',rad_to_deg*int_ned_solution(2,1)','-s','MarkerSize',5,...
    'MarkerEdgeColor','red');
hold on;
%plot end position
p3 = plot(rad_to_deg*int_ned_solution(1,:)',rad_to_deg*int_ned_solution(2,:)','y');
hold on;
plot(rad_to_deg*gnss_ned_solution(1,end)',rad_to_deg*gnss_ned_solution(2,end)','-s','MarkerSize',5,...
    'MarkerEdgeColor','green');
hold on;
plot(rad_to_deg*dr_ned_solution(1,end)',rad_to_deg*dr_ned_solution(2,end)','-s','MarkerSize',5,...
    'MarkerEdgeColor','green');
hold on;
plot(rad_to_deg*int_ned_solution(1,end)',rad_to_deg*int_ned_solution(2,end)','-s','MarkerSize',5,...
    'MarkerEdgeColor','green');
xlabel('Latitude')
ylabel('Longitude')
legend([p1 p2 p3],{'GNSS-Only','DR-Only','Integrated KF'})
title('Position Solutions for GNSS-Only, DR-Only, Integrated KF')

figure
subplot(1,2,1)
plot(time,dr_ned_solution(3,:)','r','LineWidth',0.1);
hold on;
plot(time,int_ned_solution(3,:)','y','LineWidth',0.1);
hold on;
plot(time,gnss_ned_solution(4,:)','b','LineWidth',1);
xlabel('Time (seconds)')
ylabel('Velocity North (m/s)')
legend('DR-Only','Integrated KF','GNSS Only')
title('North Velocity Solutions')
subplot(1,2,2)
plot(time,dr_ned_solution(4,:)','r','LineWidth',0.1);
hold on;
plot(time,int_ned_solution(4,:)','y','LineWidth',0.1);
hold on;
plot(time,gnss_ned_solution(5,:)','b','LineWidth',1);
xlabel('Time (seconds)')
ylabel('Velocity East (m/s)')
legend('DR-Only','Integrated KF','GNSS Only')
title('East Velocity Solutions')
end