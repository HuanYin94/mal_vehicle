
% 
% plot(mag_13_24_23(:,1), mag_13_24_23(:,2), 'r', 'LineWidth', 2);
% hold on;
% plot(loc_13_24_23(:,1), loc_13_24_23(:,2), 'k', 'LineWidth', 2);
% hold on;
% plot(odom_13_24_23(:,1)+49.96*ones(length(odom_13_24_23),1), odom_13_24_23(:,2)+11.9631*ones(length(odom_13_24_23),1), 'b', 'LineWidth', 2);
% 
% axis equal;
% grid on;
% 
% legend('mag', 'localization', 'odometry');
% xlabel('X-Meter')
% ylabel('Y-Meter')





% plot(mag_13_31_49(:,1), mag_13_31_49(:,2), 'r', 'LineWidth', 2);
% hold on;
% plot(loc_13_31_49(:,1), loc_13_31_49(:,2), 'k', 'LineWidth', 2);
% hold on;
% plot(odom_13_31_49(:,1)+49.96*ones(length(odom_13_31_49),1), odom_13_31_49(:,2)+11.9631*ones(length(odom_13_31_49),1), 'b', 'LineWidth', 2);
% 
% axis equal;
% grid on;
% 
% legend('mag', 'localization', 'odometry');
% xlabel('X-Meter')
% ylabel('Y-Meter')

load('loc_1324.txt');
load('mag_1324.txt');
load('odom_1324.txt');

plot(loc_1324(:,2), loc_1324(:,3), 'k.');
hold on;
plot(mag_1324(:,2), mag_1324(:,3), 'r.');
axis equal;
grid on;






