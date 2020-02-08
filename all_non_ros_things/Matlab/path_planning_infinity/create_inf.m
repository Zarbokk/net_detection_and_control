clc
clear

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
    angle(i) = angle_trajectory
    
end

%%
hold on
plot(waypoints_x,waypoints_y)
axis equal



current_goal=4%goal of uboot
plot(waypoints_x(current_goal),waypoints_y(current_goal), 'ro', 'MarkerSize', 10)
current_pos_boat=[rand*3,rand*1.5]
plot(current_pos_boat(1),current_pos_boat(2), 'bo', 'MarkerSize', 10)
