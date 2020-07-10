%% calibration distance general

clc
clear

data_raw = readmatrix("point_cloud.csv");
gantry_pos = readmatrix("gantry_pos.csv");
%%

for i = 1:size(gantry_pos,1)
    current_i=gantry_pos(i,4)
    current_cloud = data_raw(data_raw(:,8)==current_i,:);
    current_cloud = current_cloud(current_cloud(:,1)> -0.6 & current_cloud(:,1)< 0.6,:);
    current_cloud = current_cloud(current_cloud(:,2)> -2,:);
    median_distance(i)=-median(current_cloud(:,2));
    mean_distance(i)=-mean(current_cloud(:,2));
end

%plot3(current_cloud(:,1),current_cloud(:,2),current_cloud(:,3), '.','LineWidth',0.1)
%plot(current_cloud(:,1),current_cloud(:,2),'.')
%axis equal
%plot(mean_distance)
%%
hold on
plot(medfilt1(median_distance,10))
plot(gantry_pos(:,1))
%% plot diff
a=1.4
b=-0.6
plot(1.3-(medfilt1(median_distance,10)*a+b)-gantry_pos(:,1)')
%% test
hold on
plot(-(medfilt1(median_distance,10)*a+b)+1.3)
plot(gantry_pos(:,1))