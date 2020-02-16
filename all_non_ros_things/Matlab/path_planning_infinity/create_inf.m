clc
clear
clf
dist_y=0.7

dist_x=0.9

r=0.5

dist_circle=1.2

N=100
%while not N % 4 == 0:
%    N = N + 1

waypoints_x = zeros(N,1)
waypoints_y = zeros(N,1)

p1_x = dist_x
p1_y = dist_y + r
p2_x = dist_x + dist_circle
p2_y = dist_y - r
p3_x = dist_x + dist_circle
p3_y = dist_y + r
p4_x = dist_x
p4_y = dist_y - r

x =  linspace(0,pi,N / 4)
x_circle_r = dist_x - sin(x) * r
y_circle_r = dist_y - cos(x) * r

waypoints_x(1:N/4) = x_circle_r
waypoints_y(1:N/4) = y_circle_r

waypoints_x(N/4+1:2*N/4) = linspace(p1_x, p2_x, N / 4)
waypoints_y(N/4+1:2*N/4) = linspace(p1_y, p2_y, N / 4)

x_circle_l = dist_x + dist_circle + sin(x) * r
y_circle_l = dist_y - cos(x) * r
waypoints_x(2*N/4+1:3*N/4) = x_circle_l
waypoints_y(2*N/4+1:3*N/4) = y_circle_l
waypoints_x(3*N/4+1:4*N/4) = linspace(p3_x, p4_x, N / 4)
waypoints_y(3*N/4+1:4*N/4) = linspace(p3_y, p4_y, N / 4)

angle = zeros(N,1)
for i = 1:N
    left_point = i - 1
    right_point = i + 1
    if i == 1
        left_point = N
        right_point = i + 1
    end
    if i == N
        left_point = i - 1
        right_point = 1
    end
    angle_trajectory = atan2((-waypoints_y(left_point) + waypoints_y(right_point)),(-waypoints_x(left_point) + waypoints_x(right_point)))
    angle(i) = tan(angle_trajectory)
    
end

%%
hold on
plot(waypoints_x,waypoints_y)   
axis equal



current_goal=randi(N)%goal of uboot
current_goal=10
plot(waypoints_x(current_goal),waypoints_y(current_goal), 'ro', 'MarkerSize', 10)
current_pos_boat=[rand*3,rand*1.5]
%current_pos_boat=[1.5,0]
current_pos_boat=[2.7189,0.3195]
plot(current_pos_boat(1),current_pos_boat(2), 'bo', 'MarkerSize', 10)

%%
%end pos


y_max=waypoints_y(current_goal)-current_pos_boat(2)
x_max=waypoints_x(current_goal)-current_pos_boat(1)
%%
if x_max > 0
    x=0:0.01:x_max
    c_cubic=-atan2(-y_max,-x_max)
else    
    x=x_max:0.01:0
    c_cubic=atan2(-y_max,-x_max)
end

d_cubic=0

%b_cubic=-3*c_cubic/x_max-3*d_cubic/x_max^2+3*y_max/x_max^2+c/x_max-a/x_max
%b_cubic=(y_max-a*x_max)/(-2/3*x_max^2+x_max^2)
a_cubic=(y_max+c_cubic/2*x_max-angle(current_goal)/2*x_max-c_cubic*x_max)/(x_max^3-3/2*x_max^3)
b_cubic=(angle(current_goal)-c_cubic-3*a_cubic*x_max^2)/2/x_max
%a_cubic=-2*b_cubic/x_max/3
%a_cubic=(-b_cubic*x_max^2-c_cubic*x_max-d_cubic+y_max)/x_max^3

y=a_cubic*x.^3+b_cubic*x.^2+c_cubic*x+d_cubic
stammfunktion=a_cubic*x_max.^3+b_cubic*x_max.^2+c_cubic*x_max+d_cubic
ableitung=3*a_cubic*(x_max/2).^2+2*b_cubic*(x_max/2)+c_cubic

x=x-x_max+waypoints_x(current_goal)
y=y-y_max+waypoints_y(current_goal)

%plot(x,y)
%%
if x_max > 0 && y_max>0
    x=linspace(0,x_max,100)
    angle_rotation_matrix=-atan2(y_max,x_max)
    R=[cos(angle_rotation_matrix) -sin(angle_rotation_matrix);sin(angle_rotation_matrix) cos(angle_rotation_matrix)]
    angle_rotation_matrix=atan2(y_max,x_max)
    R_return=[cos(angle_rotation_matrix) -sin(angle_rotation_matrix);sin(angle_rotation_matrix) cos(angle_rotation_matrix)]
    tmp=R*[x_max;y_max]
    m=tan(atan(angle(current_goal))-atan2(y_max,x_max))
    x=linspace(0,tmp(1),100)
end
if x_max < 0 && y_max>0
    angle_rotation_matrix=pi-atan2(y_max,x_max)
    R=[cos(angle_rotation_matrix) -sin(angle_rotation_matrix);sin(angle_rotation_matrix) cos(angle_rotation_matrix)]
    angle_rotation_matrix=atan2(y_max,x_max)-pi
    R_return=[cos(angle_rotation_matrix) -sin(angle_rotation_matrix);sin(angle_rotation_matrix) cos(angle_rotation_matrix)]
    m=tan(atan(angle(current_goal))-atan2(y_max,x_max))
    tmp=R*[x_max;y_max]
    x=linspace(tmp(1),0,100)
end
if x_max > 0 && y_max<0
    angle_rotation_matrix=-atan2(y_max,x_max)
    R=[cos(angle_rotation_matrix) -sin(angle_rotation_matrix);sin(angle_rotation_matrix) cos(angle_rotation_matrix)]
    angle_rotation_matrix=+atan2(y_max,x_max)
    R_return=[cos(angle_rotation_matrix) -sin(angle_rotation_matrix);sin(angle_rotation_matrix) cos(angle_rotation_matrix)]
    m=tan(atan(angle(current_goal))-atan2(y_max,x_max))
    tmp=R*[x_max;y_max]
    x=linspace(0,tmp(1),100)
end
if x_max < 0 && y_max<0
    angle_rotation_matrix=pi-atan2(y_max,x_max)
    R=[cos(angle_rotation_matrix) -sin(angle_rotation_matrix);sin(angle_rotation_matrix) cos(angle_rotation_matrix)]
    angle_rotation_matrix=atan2(y_max,x_max)-pi
    R_return=[cos(angle_rotation_matrix) -sin(angle_rotation_matrix);sin(angle_rotation_matrix) cos(angle_rotation_matrix)]
    m=tan(atan(angle(current_goal))-atan2(y_max,x_max))
    tmp=R*[x_max;y_max]
    x=linspace(tmp(1),0,100)
end


a=m/tmp(1)/tmp(1)
b=-m/tmp(1)
y=a*x.^3+b*x.^2
angle_rotation_matrix=angle_rotation_matrix-pi
%R_new=[cos(angle_rotation_matrix) -sin(angle_rotation_matrix);sin(angle_rotation_matrix) cos(angle_rotation_matrix)]
x_real=R_return(1,1:2)*[x;y]+current_pos_boat(1)
y_real=R_return(2,1:2)*[x;y]+current_pos_boat(2)
plot(x_real,y_real)
%%
second_point=current_goal-2
if second_point<1
    second_point=second_point+N
end
x1=current_pos_boat(1)
x2=waypoints_x(second_point)
x3=waypoints_x(current_goal)
y1=current_pos_boat(2)
y2=waypoints_y(second_point)
y3=waypoints_y(current_goal)

A=[2*x2-2*x1 2*y2-2*y1;2*x3-2*x1 2*y3-2*y1]
c=-[x1^2-x2^2+y1^2-y2^2;x1^2-x3^2+y1^2-y3^2]
b=inv(A)*c
r=sqrt((x1-b(1))^2+(y1-b(2))^2)
th = 0:pi/50:2*pi;
xunit = r * cos(th) + b(1);
yunit = r * sin(th) + b(2);
%plot(xunit, yunit);






