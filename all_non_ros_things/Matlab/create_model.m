clc
clear

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

K=zeros(6,12);
N=16;
K_list=zeros(6,12,N*N*N);
