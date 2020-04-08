clear
clc
%states: x y z roll pitch yaw x_b y_b z_b phi theta psi
syms v x y z roll pitch yaw x_b y_b z_b phi theta psi t u thrust_input roll_input pitch_input yaw_input nothing_first nothing_second; % sym variables
v = [x;y;z;roll;pitch;yaw;x_b;y_b;z_b;phi;theta;psi]; % state vector
u = [thrust_input;nothing_first ;nothing_second ;roll_input;pitch_input;yaw_input];
A = simplify(jacobian(nonlinear_boat_model_for_linerization(t,v,u), v)); % calc jacobi
B = simplify(jacobian(nonlinear_boat_model_for_linerization(t,v,u), u)); % calc jacobi

A_fun = matlabFunction(A,'File','Linerize_A','Vars',{v,u}); % write to file
B_fun = matlabFunction(B,'File','Linerize_B','Vars',{v,u}); % write to file
%%
%example lqr calculation

Q=diag([1 1 1 100 10 10 1 1 1 1000 1000 1000]);
R=eye(6)*10;
lin_A=Linerize_A([0;0;0;0;0;0;0;0;0;0;0;0],0)
lin_B=Linerize_B([0;0;0;0;0;0;0;0;0;0;0;0],0)   

length(lin_A)-rank(ctrb(lin_A,lin_B))

[K,S,P] = lqr(lin_A,lin_B,Q,R);
K

%% Beispiel explicit euler
clc
clear
h = 0.01; % step size
t_max = 10; % max time
t = 0:h:t_max; % time vector
Rb = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]; % initial value

x = zeros(length(t), length(Rb));% state vector
x(1, :) = Rb; % initial condition
u = zeros(length(t), 6); % system input

% explicit euler
for i = 2:length(t)
    if t(i)>5 && t(i)<6
        u(i-1,:)=[0 0 0 1 0 0];
    end
   x(i, :) = x(i-1, :)' + h * nonlinear_boat_model(t(i-1), x(i-1, :)', u(i-1,:)');
end


plot(t,x(:,4))