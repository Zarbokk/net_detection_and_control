clc
clear

data_raw = readmatrix("px4_pos.csv");
%plot(tmp.v(:,1),tmp.v(:,3), 'o')
data_raw =data_raw(3000:end,:);
error=data_raw(:,4);
x=data_raw(:,1);
y=data_raw(:,2);
z=data_raw(:,3);
time=data_raw(:,5);

%% read in data of cylinder
%cylinder=readObj("World_with_cylinder_net.obj");
cylinder=readObj("World_with_oval_complicated_net.obj");
position_net_x=cylinder.v(:,1);
position_net_y=cylinder.v(:,3);


%% calculation error 
distance_to_net=ones(size(data_raw,1),2);


 position_net_x=interp(position_net_x,10);
 position_net_y=interp(position_net_y,10);
for i = 1:size(data_raw,1)
    distance_to_net(i,1)=min(sqrt((x(i)-position_net_x).*(x(i)-position_net_x)+(y(i)-position_net_y).*(y(i)-position_net_y)));
end


%%
f1=figure(1);
set(gcf,'Position',[100 100 500 300])
hold on 
plot(x, y,'LineWidth',1.5)
%plot(position_net_x,position_net_y)
fill(position_net_x, position_net_y,'g')
xlabel('x-axis in m','interpreter','latex')
ylabel('y-axis in m','interpreter','latex')
axis([-6 8 -4 4])
grid on
%axis equal
legend({'Position \muAUV','Aquaculture Net'},'FontSize',9,'Location','northeast')
hold off
print(f1,'net_with_position.pdf','-dpdf','-r0')
system('pdfcrop net_with_position.pdf net_with_position.pdf');
%%
f2=figure(2);
set(gcf,'Position',[100 100 500 300])
line_width=2.0;
hold on 
plot(time, distance_to_net(:,1),'LineWidth',line_width)
desired_distance = ones(size(time),'like',time)*1.5;%1.5 meter
plot(time, desired_distance,'LineWidth',line_width)
xlabel('time in s','interpreter','latex')
ylabel('Distance to Aquaculture Net','interpreter','latex')
%axis([-4 5 -4.5 4.5])
grid on
%axis equal
legend({'\muAUV distance','desired distance'},'FontSize',10)
hold off
print(f2,'error_to_net.pdf','-dpdf','-r0')
system('pdfcrop error_to_net.pdf error_to_net.pdf');



