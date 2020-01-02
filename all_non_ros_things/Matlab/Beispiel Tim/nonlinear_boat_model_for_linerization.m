function v_d = nonlinear_boat_model_for_linerization(t,v,u)

%u=u*t
%NONLINEAR_BOAT_MODEL Summary of this function goes here
%   Detailed explanation goes here
%load variables

l=0.305;
d=0.235;
m=1.47;
B=14.42;
I_x=0.002408;
I_y=0.010717;
I_z=I_y;
X_u_d=-1.11;
Y_v_d=-2.8;
Z_w_d=-2.8;
K_p_d=-0.00451;
M_q_d=-0.0163;
N_r_d=-0.0163;
X_u_u=-5.39;
Y_v_v=-17.36;
Z_w_w=-17.36;
K_p_p=-0.00114;
M_q_q=-0.007;
N_r_r=-0.007;
%D=0.05
C_T=0.0000363;
C_D=2.469*10^-9;
%k_t=
%k_m=

h=0.0481*2;

k_m=C_D/C_T;

F_to_xrpy=[1 1 1 1;-k_m k_m -k_m k_m ; -h/2 -h/2 h/2 h/2 ; h/2 -h/2 -h/2 h/2];


%xrpy_to_F=inv(F_to_xrpy)


mix_matrix_trpy_to_velocity_motor=[[1 1 1 1]' [-1 1 -1 1]' [-1 -1 1 1]' [1 -1 -1 1]'];%inv([1 1 1 1;1 -1 1 -1 ; -1 -1 1 1 ; 1 -1 -1 1])



D_l=-diag([X_u_u Y_v_v Z_w_w K_p_p M_q_q N_r_r]);
D_r=-diag([X_u_u Y_v_v Z_w_w K_p_p M_q_q N_r_r]);

M_RB=diag([m m m I_x I_y I_z]);
M_A=-diag([X_u_d Y_v_d Z_w_d K_p_d M_q_d N_r_d]);

M_RB_11=M_RB(1:3,1:3);
M_RB_22=M_RB(4:6,4:6);
M_A_11=M_A(1:3,1:3);
M_A_22=M_A(4:6,4:6);

M_RB_M_A_INV=inv(M_RB+M_A);


%% SIMULATE 
%states: x y z roll pitch yaw x_b y_b z_b phi theta psi
%
%display(v);

%mix_matrix_trpy_to_velocity_motor=[[1 1 1 1]' [1 -1 1 -1]' [-1 -1 1 1]' [1 -1 -1 1]'];

pwm_signal=9*mix_matrix_trpy_to_velocity_motor*[u(1) u(4) u(5) u(6)]';
% for i=1:4
%     if pwm_signal(i)>500
%         pwm_signal(i)=500;
%     end
%     if pwm_signal(i)<-500
%         pwm_signal(i)=-500;
%     end
% end

%F=pwm_signal.*abs(pwm_signal)*C_T;
%F=F/max(F)*7
Real_trpy=F_to_xrpy*pwm_signal;
u(1)=Real_trpy(1);
u(4)=Real_trpy(2);
u(5)=Real_trpy(3);
u(6)=Real_trpy(4);

%display(v)
C_RB=[zeros(3) -m*cross(repmat([v(7) v(8) v(9)]',1,3),eye(3,3));-m*cross(repmat([v(7) v(8) v(9)]',1,3),eye(3,3)) cross(repmat(M_A_22*[v(10) v(11) v(12)]',1,3),eye(3,3))];
C_A=[zeros(3) -m*cross(repmat(M_A_11*[v(7) v(8) v(9)]',1,3),eye(3,3));-m*cross(repmat(M_A_11*[v(7) v(8) v(9)]',1,3),eye(3,3)) cross(repmat(M_A_22*[v(10) v(11) v(12)]',1,3),eye(3,3))];

%%

%u_1 = interp1(ut,u_1,t)
%u_2 = interp1(ut,u_2,t)
%u_3 = interp1(ut,u_3,t)
%u_4 = interp1(ut,u_4,t)

v_d(4:6)=[1 0 +sin(v(5));0 cos(v(4)) -sin(v(4))+sin(v(5));0 sin(v(4)) cos(v(4))]*[v(10) v(11) v(12)]';
v_d(1:3)=[v(7) v(8) v(9)]';

v_d(7:12)=M_RB_M_A_INV*(-C_A*v(7:12)-C_RB*v(7:12)-D_l*v(7:12)+u);
v_d=v_d';
end

