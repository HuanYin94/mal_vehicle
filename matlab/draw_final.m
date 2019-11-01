
load('loc_1324.txt');
load('mag_1324.txt');
load('odom_1324.txt');

for i=1:length(odom_1324)
   odom_1324(i,2:4) = odom_1324(i,2:4) + mag_1324(1,2:4); 
end

plot(loc_1324(:,2), loc_1324(:,3), 'k.');
hold on;
plot(mag_1324(:,2), mag_1324(:,3), 'r.');
hold on;
plot(odom_1324(:,2), odom_1324(:,3), 'b.');
axis equal;
grid on;

legend('EKF', 'Mag', 'Odom');

xlabel('X-Meter')
ylabel('Y-Meter')










% 
% 
% 
% load('loc_1331.txt');
% load('mag_1331.txt');
% load('odom_1331.txt');
% 
% for i=1:length(odom_1331)
%    odom_1331(i,2:4) = odom_1331(i,2:4) + mag_1331(1,2:4); 
% end
% 
% plot(loc_1331(:,2), loc_1331(:,3), 'k.');
% hold on;
% plot(mag_1331(:,2), mag_1331(:,3), 'r.');
% hold on;
% plot(odom_1331(:,2), odom_1331(:,3), 'b.');
% axis equal;
% grid on;
% 
% legend('EKF', 'Mag', 'Odom');
% 
% xlabel('X-Meter')
% ylabel('Y-Meter')
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
