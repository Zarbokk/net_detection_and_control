C_T=0.0000363
C_D=2.469*10^-9
D=0.05
h=D

k_m=C_D/C_T*D

F_to_xrpy=[1 1 1 1;k_m -k_m k_m -k_m ; -h/2 -h/2 h/2 h/2 ; h/2 -h/2 -h/2 h/2]


xrpy_to_F=inv(F_to_xrpy)


xrpy_to_F*[1 0.001 0 0.001]'

rpy=F_to_xrpy*[1 0 0 0]'