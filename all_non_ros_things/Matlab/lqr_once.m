
mdl='just_model';


opspec = operspec(mdl); 
opspec.States(7).Known = 1;
opspec.States(7).x = 0/180*pi;%roll
opspec.States(8).Known = 1;
opspec.States(8).x = 0/180*pi;%pitch
opspec.States(9).Known = 1;
opspec.States(9).x = 0/180*pi;%yaw
options = findopOptions('DisplayReport','off');
op = findop(mdl,opspec,options);
sys_lin=linearize(mdl,op);
%length(sys_lin.A)-rank(ctrb(sys_lin));
%length(sys_lin.A)-rank(obsv(sys_lin));


Q=diag([1 1000 1000 1000 100 10 10 1]);
R=eye(4)*10;
[K,S,P] = lqr(sys_lin,Q,R);
K
