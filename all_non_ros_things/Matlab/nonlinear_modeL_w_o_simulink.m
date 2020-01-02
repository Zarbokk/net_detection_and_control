%%
clc
h = 0.001; % step size
t_max = 10; % max time
t = 0:h:t_max; % time vector
Rb = [0, 0, 0, -0/180*pi, 0/180*pi, 0/180*pi, 0, 0, 0, 0, 0, 0]; % initial value

x = zeros(length(t), length(Rb));% state vector
x(1, :) = Rb; % initial condition
u = zeros(length(t), 6); % system input
%states: x y z roll pitch yaw x_b y_b z_b phi theta psi
% explicit euler
Q=diag([1 1 1 10 10 10 1 1 1 100 100 100]);
R=eye(6)*10;
lin_A=Linerize_A([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]',0);
lin_B=Linerize_B([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]',0);
[K,S,P] = lqr(lin_A,lin_B,Q,R);

states_wanted=[0, 0, 0, -20/180*pi, 20/180*pi, 170/180*pi, 0, 0, 0, 0, 0, 0]


for i = 2:length(t)
    tic
    lin_A=Linerize_A(x(i-1, :)',0);
    lin_B=Linerize_B(x(i-1, :)',0);
    
    %length(lin_A)-rank(ctrb(lin_A,lin_B))
    %x(i-1, :)'
    
    [K,S,P] = lqr(lin_A,lin_B,Q,R);
    toc
    input=-K*(x(i-1, :)-states_wanted)';%x y z roll pitch yaw x_b y_b z_b phi theta psi
    u(i,:)=input';
    x(i, :) = x(i-1, :)' + h * nonlinear_boat_model(t(i), x(i-1, :)', u(i,:)');
    i/length(t)
end


%length(lin_A)-rank(ctrb(lin_A,lin_B))




hold on
plot(t,x(:,4)*180/pi)
plot(t,x(:,5)*180/pi)
plot(t,x(:,6)*180/pi)
legend('roll','pitch','yaw')
%%
hold on
plot(t,u(:,1))
plot(t,u(:,4))
plot(t,u(:,5))
plot(t,u(:,6))
legend('thrust','roll input','pitch input','yaw input')
%%

plot(t,x(:,1))