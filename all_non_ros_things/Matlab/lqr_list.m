
N=16;
i=1;
K_list=zeros(6,12,N*N*N);
for roll = linspace(0,2*pi,N)
    for pitch = linspace(0,2*pi,N)
        for yaw = linspace(0,2*pi,N)
            operating_point=[0, 0, 0, roll, pitch, yaw, 0, 0, 0, 0, 0, 0]';
            %length(sys_lin.A)-rank(ctrb(sys_lin));
            %length(sys_lin.A)-rank(obsv(sys_lin));


            Q=diag([1 1 1 100 10 10 1 1 1 1000 100 100]);%states: x y z roll pitch yaw x_b y_b z_b phi theta psi
            R=eye(6)*10;
            lin_A=Linerize_A(operating_point,0);
            lin_B=Linerize_B(operating_point,0);
            %length(lin_A)-rank(ctrb(lin_A,lin_B))
            %x(i-1, :)'
            [K,S,P] = lqr(lin_A,lin_B,Q,R);
            K_list(:,:,i)=K;
            i=i+1
        end
    end
end

writematrix(K_list,'tmp.csv');
