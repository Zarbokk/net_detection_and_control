clc
clear

%ebene:
c=-30
a=-30%steigung x
f=476

x_real=400/f*abs(c)
a=a/abs(x_real)*0.5
d=sqrt((x_real*a)^2 + x_real^2)
dist_des=30
e=sqrt(d^2+dist_des^2)
alpha=pi-pi/2-atan(abs(a))
gamma=asin(dist_des*sin(pi/2)/e)

betta=pi-gamma-alpha
betta*180/pi


y_max=c+e*cos(betta)
x_max=sin(betta)*e


d_cubic=0
c_cubic=a
%b_cubic=-3*c_cubic/x_max-3*d_cubic/x_max^2+3*y_max/x_max^2+c/x_max-a/x_max
b_cubic=(y_max-a*x_max)/(-2/3*x_max^2+x_max^2)
a_cubic=-2*b_cubic/x_max/3
%a_cubic=(-b_cubic*x_max^2-c_cubic*x_max-d_cubic+y_max)/x_max^3
x=0:0.1:x_max
test=a_cubic*x.^3+b_cubic*x.^2+c_cubic*x+d_cubic
stammfunktion=a_cubic*x_max.^3+b_cubic*x_max.^2+c_cubic*x_max+d_cubic
ableitung=3*a_cubic*x_max.^2+2*b_cubic*x_max+c_cubic

hold on 
plot(x,test)
x=-x_max:0.1:x_max
plot(x,a*x+c)
%test=3*a_cubic*x.^2+2*b_cubic*x+c_cubic


