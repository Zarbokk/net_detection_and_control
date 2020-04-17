clc
clear

data_raw = readmatrix("px4_pos.csv");
data_raw =data_raw(3000:end,:);
error=data_raw(:,4);
x=data_raw(:,1);
y=data_raw(:,2);
z=data_raw(:,3);
time=data_raw(:,5);

%%
angle=0:0.1:pi*2+0.1;
r=2;
position_net_x=r*cos(angle);
position_net_y=r*sin(angle);
%%
f1=figure(1);
hold on 
plot(x, y,'LineWidth',1.5)
%plot(position_net_x,position_net_y)
fill(position_net_x, position_net_y,'g')
xlabel('x-axis in m')
ylabel('y-axis in m')
axis([-4 5 -4.5 4.5])
grid on
axis equal
legend({'Position \muAUV','Aquaculture Net'},'FontSize',12)
hold off
print(f1,'net_with_position.pdf','-dpdf','-r0')
system('pdfcrop net_with_position.pdf net_with_position.pdf');
%%
f2=figure(2);
line_width=2.0;
hold on 
plot(time, error+1.5,'LineWidth',line_width)
desired_distance = ones(size(time),'like',time)*1.5;%1.5 meter
plot(time, desired_distance,'LineWidth',line_width)
xlabel('time in s')
ylabel('Distance to Aquaculture Net')
%axis([-4 5 -4.5 4.5])
grid on
%axis equal
legend({'\muAUV distance','desired distance'},'FontSize',12)
hold off
print(f2,'error_to_net.pdf','-dpdf','-r0')
system('pdfcrop error_to_net.pdf error_to_net.pdf');