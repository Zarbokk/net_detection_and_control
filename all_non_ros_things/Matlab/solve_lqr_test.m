clc
clear
operating_point=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]';
Q=diag([1 1 1 100 10 10 1 1 1 1000 100 100]);%states: x y z roll pitch yaw x_b y_b z_b phi theta psi
R=eye(6)*10;
A=Linerize_A(operating_point,0);
B=Linerize_B(operating_point,0);


H=[A,-B*inv(R)*B';-Q , -A'];
%length(lin_A)-rank(ctrb(lin_A,lin_B))
%x(i-1, :)'
%[K,S,P] = lqr(lin_A,lin_B,Q,R);

H_local=H
iterations=0
converged=false;
while ~converged
    if iterations>10
        break
    end
    
    H_diff=H_local-inv(H_local);
    H_new=H_local-0.5*H_diff;
                    %see if converged
    norm(H_local-H_new)
    H_local=H_new;
    
    iterations=iterations+1;
end
%H_local-H_new
schur(H)