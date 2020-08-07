clc
clear

data_raw = readmatrix("point_cloud.csv");
gantry_pos = readmatrix("gantry_pos.csv");

%hold on
%plot(data_raw(:,1),data_raw(:,2), 'o')
%axis equal
%%

current_cloud = data_raw(data_raw(:,8)==505,:);
current_cloud = current_cloud(current_cloud(:,1)> -0.5 & current_cloud(:,1)< 0.5,:);
current_cloud = current_cloud(current_cloud(:,2)> -1.5,:);


plot3(current_cloud(:,1),current_cloud(:,2),current_cloud(:,3), 'o')
axis equal
%% remove unnecessary points


%% x y z 

hold on
plot3(current_cloud(:,1),current_cloud(:,2),current_cloud(:,3), 'o')
axis equal


%estimate x=a*y^2+b*z^2+c*x+d
%[x,resnorm,residual,exitflag,output] = lsqnonlin(@(x) tmp(x,x_pos,y_pos,z_pos,d),x0);
scaling=1.6;
delta=0;
y = current_cloud(:,2);
x = current_cloud(:,1);
z = current_cloud(:,3);
fun = @(ab)ab(1)*x.^2+ab(2)*z.^2-y+ab(3);
x0 = [1 1 1];%example -0.4727    0.0769
x0_optimated = lsqnonlin(fun,x0)
x0_optimated(1)=x0_optimated(1);
x0_optimated(2)=x0_optimated(2)*0;
%%
%x0_optimated =

%    0.0481   -0.1396   -1.2042
x_plotting=linspace(-0.5,0.5);
z_plotting=linspace(-0.5,0.5);
y_plotting=x0_optimated(1)*x_plotting.^2+x0_optimated(2)*z_plotting.^2+x0_optimated(3);
hold on
plot3(x_plotting,y_plotting,z_plotting)
plot3(current_cloud(:,1),current_cloud(:,2),current_cloud(:,3), 'o')
%%

data_raw_new=current_cloud;
for i=1:size(data_raw_new,1)
    data_raw_new(i,1);
    change=x0_optimated(1)*data_raw_new(i,1)^2+x0_optimated(2)*data_raw_new(i,3)^2;
    data_raw_new(i,2)=data_raw_new(i,2)-change;
end
plot3(data_raw_new(:,1),data_raw_new(:,2),data_raw_new(:,3), 'o','Color','r')
axis equal

