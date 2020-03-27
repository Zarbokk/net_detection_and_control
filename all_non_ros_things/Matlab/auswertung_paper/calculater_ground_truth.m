clc
clear
%folder_name='11_test_scaling_half_tags';
folder_name='11_test_scaling';
which_test='10';
gantry_z = 0.0; % soll - 10cm 
% scaling_x=1.05*3.1/3.1245;
% scaling_y=1*1.6/1.52;
% scaling_z=1;
% bias_x=-0.12+0.018;
% bias_y=-0.047;
%holz to kamera line = 4.5 cm 
scaling_x=1;
scaling_y=1;
scaling_z=1;
bias_x=0;
bias_y=0;

estimated_data_raw = readmatrix(folder_name+"/"+which_test+"/estimated_pose.csv");
gantry_data_raw = readmatrix(folder_name+"/"+which_test+"/gantry_pose.csv");
pixracer_data_raw = readmatrix(folder_name+"/"+which_test+"/pixracer_pose.csv");
estimated_data_raw(:,1) = estimated_data_raw(:,1) - estimated_data_raw(1,1);
gantry_data_raw(:,1) = gantry_data_raw(:,1) - gantry_data_raw(1,1);
pixracer_data_raw(:,1) = pixracer_data_raw(:,1) - pixracer_data_raw(1,1);


[gantry_data_raw(1:5,1) - gantry_data_raw(1,1) estimated_data_raw(1:5,1) - estimated_data_raw(1,1)  pixracer_data_raw(1:5,1) - pixracer_data_raw(1,1)]

item = 5;

gantry_counter = 1;

compare_data = pixracer_data_raw;
for estimated_counter = 1:size(compare_data,1)
    gantry_time = gantry_data_raw(gantry_counter,1);
    estimated_time = compare_data(estimated_counter,1);
    
    if gantry_time < estimated_time
        gantry_counter = gantry_counter + 1;
    end
    if gantry_counter > size(gantry_data_raw,1)-10
        break
    end
    gantry_data_aligned(estimated_counter,:) = [gantry_data_raw(gantry_counter,7), gantry_data_raw(gantry_counter,8)];
    compare_data_aligned(estimated_counter,:) = [compare_data(estimated_counter,5), compare_data(estimated_counter,6), compare_data(estimated_counter,7)];
    
    latency = 13;
    error_x(estimated_counter) = gantry_data_raw(gantry_counter,7) - compare_data(estimated_counter+latency, 5);
    error_y(estimated_counter) = gantry_data_raw(gantry_counter,8) - compare_data(estimated_counter+latency, 6);
    error_z(estimated_counter) = gantry_z - compare_data(estimated_counter+latency, 7);
end

% mean_x = mean(abs(error_x))
% mean_y = mean(abs(error_y))
% mean_z = mean(abs(error_z))
% std_x = std(error_x)
% std_y = std(error_y)
% std_z = std(error_z)
%%
figure(1)
hold on 
plot(gantry_data_aligned(:,1), gantry_data_aligned(:,2))
plot(compare_data_aligned(:,1),compare_data_aligned(:,2))
xlabel('x-axis in m')
ylabel('y-axis in m')
axis([-0.2 3.5 -0.2 2])
grid on
hold off
%%

% figure(2)
% hold on
% %plot(error_x)
% %plot(error_y)
% plot(error_z)
% hold off
% 
% figure(3)
% hold on
% plot(gantry_data_aligned(:,1))
% plot(compare_data_aligned(:,1))
% hold off

%%
estimated_x=estimated_data(:,5)*scaling_x+bias_x;
estimated_y=estimated_data(:,6)*scaling_y+bias_y;
estimated_z=estimated_data(:,7);

gantry_pos_x=gantry_data(:,7);
gantry_pos_y=gantry_data(:,8);
pixracer_pos_x=pixracer_data(:,5);
pixracer_pos_y=pixracer_data(:,6);
pixracer_pos_z=pixracer_data(:,7);
hold on
title(which_test+"cm under the water surface")
plot(estimated_x,estimated_y)
plot(pixracer_pos_x,pixracer_pos_y)
plot(gantry_pos_x,gantry_pos_y)
legend('estimated','pixracer','gantry')
ylabel('position y in m')
xlabel('position x in m')
xlim([0 3.5])
axis equal

%%
%N = 5; %// Number of points to introduce in between each control point
%y = [0 0 1 0 0]; %// Your output data
%L = numel(y); %// Size of output data. Cache so we don't have to keep typing in numel(y)
%x = 1:L; %// Dummy vector
%L=size(gantry_pos_x,1)
%xp = linspace(1, L, N*(L-1) + N); %// Create points to interpolate. N*(L-1) + N is also just N*L
%out = interp1(gantry_pos_x, gantry_pos_y, xp, 'linear')

clc
dist_x=zeros(size(estimated_x,1),1);
dist_y=zeros(size(estimated_y,1),1);
dist_z=zeros(size(estimated_z,1),1);
for i = 1:size(estimated_x,1)
    dist_x(i) = min(abs(gantry_pos_x-estimated_x(i)));
    dist_y(i) = min(abs(gantry_pos_y-estimated_y(i)));
    dist_z(i) = min(abs(0-estimated_z(i)));
end


e_mean = [mean(dist_x) mean(dist_y) mean(dist_z)]
e_std = [std(dist_x) std(dist_y) std(dist_z)]
