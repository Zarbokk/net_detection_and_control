function dx = Doppelpendel_DGL(t,x)
 
dx = [x(3); x(4);
      (1*(-2*9.81*sin(x(1))-x(4)^2*sin(x(1)-x(2)))-cos(x(1)-x(2))*(-9.81*sin(x(2))+x(3)^2*sin(x(1)-x(2))))/(2-(cos(x(1)-x(2)))^2);
      (-cos(x(1)-x(2))*(-2*9.81*sin(x(1))-x(4)^2*sin(x(1)-x(2)))+2*(-9.81*sin(x(2))+x(3)^2*sin(x(1)-x(2))))/(2-(cos(x(1)-x(2)))^2)];
end