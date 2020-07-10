clc
clear

data_raw = readmatrix("point_cloud.csv");
gantry_pos = readmatrix("gantry_pos.csv");

%hold on
%plot(data_raw(:,1),data_raw(:,2), 'o')
%axis equal


%% x y z 

hold on
plot3(data_raw(:,1),data_raw(:,2),data_raw(:,3), 'o')



%estimate x=a*y^2+b*z^2+c*x+d
%[x,resnorm,residual,exitflag,output] = lsqnonlin(@(x) tmp(x,x_pos,y_pos,z_pos,d),x0);
scaling=1.6;
delta=0
y = data_raw(:,2);
x = data_raw(:,1);
z = data_raw(:,3);
fun = @(ab)ab(1)*y.^2+ab(2)*z.^2-x+ab(3);
x0 = [1 1 1];%example -0.4727    0.0769
x0_optimated = lsqnonlin(fun,x0)
%%
y_plotting=linspace(-0.3,0.3);
z_plotting=linspace(-0.3,0.3);
x_plotting=x0_optimated(1)*y_plotting.^2+x0_optimated(2)*z_plotting.^2+x0_optimated(3);
plot3(x_plotting,y_plotting,z_plotting)
data_raw_new=data_raw;
for i=1:size(data_raw_new,1)
    data_raw_new(i,1)
    change=x0_optimated(1)*data_raw_new(i,2)^2+x0_optimated(2)*data_raw_new(i,3)^2;
    data_raw_new(i,1)=data_raw_new(i,1)-change;
end
plot3(data_raw_new(:,1),data_raw_new(:,2),data_raw_new(:,3), 'o','Color','r')
axis equal

