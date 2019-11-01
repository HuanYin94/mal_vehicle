% clear;clc;
% close all;
% load('loc_1324.txt');
% load('mag_1324.txt');
% load('odom_1324.txt');
% 
% mag_time = mag_1324(:,1);
% loc_time = loc_1324(:,1);
% 
% kdTree = KDTreeSearcher(mag_time, 'Distance', 'euclidean');
% 
% for i=1:length(loc_1324)
%     
%     [Idx, Dis] = knnsearch(kdTree, loc_time(i, :), 'Distance', 'euclidean', 'K', 1);
%     
%     error_xy(i,:) = norm(loc_1324(i,2:3) - mag_1324(Idx,2:3));
%     error_yaw(i,:) = rad2deg(abs(loc_1324(i,4) - mag_1324(Idx,4)));
% 
% %     error_xy(i,:) = loc_1324(i,2:3) - mag_1324(Idx,2:3);
% %     error_yaw(i,:) = rad2deg(loc_1324(i,4) - mag_1324(Idx,4));
% end
% 
% figure;
% plot(error_xy);
% grid on;
% xlabel('Pose N');
% ylabel('XY Error (m)');
% 
% disp('Trans-Error')
% disp(mean(error_xy));
% 
% figure;
% plot(error_yaw);
% grid on;
% xlabel('Pose N');
% ylabel('Yaw Error (deg)');
% 
% disp('Rot-Error')
% disp(mean(error_yaw))






clear;clc;
close all;
load('loc_1331.txt');
load('mag_1331.txt');
load('odom_1331.txt');

mag_time = mag_1331(:,1);
loc_time = loc_1331(:,1);

kdTree = KDTreeSearcher(mag_time, 'Distance', 'euclidean');

for i=1:length(loc_1331)

    [Idx, Dis] = knnsearch(kdTree, loc_time(i, :), 'Distance', 'euclidean', 'K', 1);
    
    error_xy(i,:) = norm(loc_1331(i,2:3) - mag_1331(Idx,2:3));
    error_yaw(i,:) = rad2deg(abs(loc_1331(i,4) - mag_1331(Idx,4)));
end

figure;
plot(error_xy);
grid on;
xlabel('Pose N');
ylabel('XY Error (m)');

disp('Trans-Error')
disp(mean(error_xy));

figure;
plot(error_yaw);
grid on;
xlabel('Pose N');
ylabel('Yaw Error (deg)');

disp('Rot-Error')
disp(mean(error_yaw))

