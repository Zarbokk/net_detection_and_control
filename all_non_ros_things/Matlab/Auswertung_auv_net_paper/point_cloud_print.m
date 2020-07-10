clc
clear

data_raw = readmatrix("point_cloud.csv");
%data_raw =data_raw(3000:end,:);
data_raw =data_raw(abs(data_raw(:,1))<1.5,:);
data_raw =data_raw(data_raw(:,1)<-0.05,:);
data_raw =data_raw(abs(data_raw(:,2))<1.5,:);
data_raw =data_raw(abs(data_raw(:,3))<1.5,:);
data_raw =data_raw(data_raw(:,2)<-0.3,:);
data_raw =data_raw(abs(data_raw(:,8))<3,:);
x=data_raw(:,2);
y=data_raw(:,3);
z=data_raw(:,1);
r=data_raw(:,4);
g=data_raw(:,5);
b=data_raw(:,6);
distance_camera=vecnorm(data_raw(:,1:3)')';
time=data_raw(:,7);

%%
f1=figure(1);
tiledlayout(1,2)
nexttile
hold on 
%f1=figure(2);
set(gcf,'Position',[100 100 500 200])
%hold on 
%plot(x, y,'LineWidth',1.5)
%scatter(x, y,2,'filled')
scatter(y,-z,2,distance_camera)
plot([-0.25 0.35],[0.13,0.13],'r')
%c = colorbar;
%c.Label.String = 'Distance to net';
%plot(position_net_x,position_net_y)
set(gca, 'YDir','reverse')
xlabel('x-axis in m','interpreter','latex')
ylabel('y-axis in m','interpreter','latex')
%axis([-6 8 -4 4])

axis([-0.5 0.5 -0.1 0.5])
grid on


%%
start=-0.11;
distance=0.05;
data_raw =data_raw(data_raw(:,1)<start,:);
data_raw =data_raw(data_raw(:,1)>start-distance,:);
x=data_raw(:,2);
y=data_raw(:,3);
z=data_raw(:,1);
r=data_raw(:,4);
g=data_raw(:,5);
b=data_raw(:,6);
distance_camera=vecnorm(data_raw(:,1:3)')';
time=data_raw(:,7);
%%
nexttile

%f1=figure(1);
%set(gcf,'Position',[100 100 500 300])
hold on 
%plot(x, y,'LineWidth',1.5)
%scatter(x, y,2,'filled')
%h1 = axes

scatter(-x,y,2,distance_camera)
plot(0,0,'r.','MarkerSize',20)

set(gca, 'XDir','reverse')
txt = text(0.08, -0.21, ("Depth-Camera"),'Interpreter','latex');
txt.FontSize = 7;
set(txt,'Rotation',90);
axis([ 0 1.1 -0.5 0.5])
%plot(position_net_x,position_net_y)
xlabel('z-axis in m','interpreter','latex')
ylabel('x-axis in m','interpreter','latex')
%plot([0.8,0.8],[-0.25 0.35],'r')
grid on
%axis equal
%hold off
%print(f1,'net_with_position.pdf','-dpdf','-r0')
%system('pdfcrop net_with_position.pdf net_with_position.pdf');

c = colorbar;
c.Label.String = 'Distance';
%axis equal
%hold off
print(f1,'depth_image.pdf','-dpdf','-r0')
system('pdfcrop depth_image.pdf depth_image.pdf');