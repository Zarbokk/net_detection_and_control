clc
clear
load('K_list_lqr_moment.mat')

roll=0
pitch=pi*3/4
yaw=1*pi*3/4





N=16;
roll_list = linspace(0,2*pi,N);
pitch_list = linspace(0,2*pi,N);
yaw_list = linspace(0,2*pi,N);

[M,index_roll]=min(abs(roll_list-roll));
[M,index_pitch]=min(abs(pitch_list-pitch));
[M,index_yaw]=min(abs(yaw_list-yaw))
index=(index_roll-1)*64+(index_pitch-1)*8+index_yaw;
yaw_index = (yaw/(2.0*3.14159/(15.0))+0.5)
%pitch_index = (pitch_body/(2.0*3.14159/(7.0)));
%roll_index = (roll_body/(2.0*3.14159/(7.0)));
%index=(roll_index)*64+(pitch_index)*8+yaw_index;
K_list(:,:,index)
%%
roll=0/180*pi
pitch=0/180*pi
yaw=90/180*pi
alpha = yaw
beta = pitch
gamma = roll
R_roll=[1 0 0;0 cos(roll) -sin(roll);0 sin(roll) cos(roll)]
R_pitch=[cos(pitch) 0 sin(pitch);0 1 0; -sin(pitch) 0 cos(pitch)]
R_yaw=[cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1]

R=R_yaw*R_pitch*R_roll

R=[cos(alpha)*cos(beta) cos(alpha)*sin(beta)*sin(gamma)-sin(alpha)*cos(gamma) cos(alpha)*sin(beta)*cos(gamma)+sin(alpha)*sin(gamma);
  sin(alpha)*cos(beta) sin(alpha)*sin(beta)*sin(gamma)+cos(alpha)*cos(gamma) sin(alpha)*sin(beta)*cos(gamma)-cos(alpha)*sin(gamma);
  -sin(beta) cos(beta)*sin(gamma) cos(beta)*cos(gamma)]


R_roll*[0 10 0]'


%%

motor1=sqrt(20);%pwm
motor2=sqrt(20);
motor3=sqrt(20);%pwm
motor4=sqrt(20);
C=2;

F1=motor1*motor1*C
F2=motor2*motor2*C
F3=motor3*motor3*C
F4=motor4*motor4*C
F_ges=F1+F2+F3+F4


motor1=sqrt(20)+sqrt(20);%pwm
motor2=sqrt(20)-sqrt(20);
motor3=sqrt(20)+sqrt(20);%pwm
motor4=sqrt(20)-sqrt(20);

F1=motor1*motor1*C
F2=motor2*motor2*C
F3=motor3*motor3*C
F4=motor4*motor4*C
F_ges=F1+F2+F3+F4

%%


thrust=0.1
roll=0
pitch=0
yaw=0.5



vector=[thrust roll pitch yaw]'

%mixer_matrix_to_pwm=[1 1 1 1;-1 1 -1 1 ; -1 -1 1 1;1 -1 -1 1]'
mix_matrix_trpy_to_velocity_moto=[[1 1 1 1]' [-1 1 -1 1]' [-1 -1 1 1]' [1 -1 -1 1]']

pwm_signal=500*mix_matrix_trpy_to_velocity_motor*vector

Vergleich_gazebo=pwm_signal+1500

for i=1:4
    if pwm_signal(i)>500
        pwm_signal(i)=500;
    end
    if pwm_signal(i)<-500
        pwm_signal(i)=-500;
    end
end


F=pwm_signal.*abs(pwm_signal)*C_T
%F=F/max(F)*7
Real_trpy=F_to_xrpy*F

%inv(mixer_matrix_to_pwm)*((mixer_matrix_to_pwm*vector).*(mixer_matrix_to_pwm*vector))

