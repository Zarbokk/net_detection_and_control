clc
clear
load('pose_and_covarianz.mat')
beginning_pos=pose_and_covarianz(1,1:3);
number_of_points=85
complete_poses=pose_and_covarianz(1:number_of_points,1:3);
end_pos=pose_and_covarianz(number_of_points,1:3);
time=pose_and_covarianz(1:number_of_points,4);
a_0=current_imu_accel(1:(number_of_points),:);
for i = 2:number_of_points
    
    if pose_and_covarianz(i,5)>1
        information_matrix(i-1,1)=1/pose_and_covarianz(i,5);
        z(i-1,1:3)=pose_and_covarianz(i,1:3)-pose_and_covarianz(i-1,1:3);
        %lb(i,1:3)=-[10 10 10];
        %ub(i,1:3)=+[10 10 10];
    else
        z(i-1,1:3)=[0 0 0];%pose_and_covarianz(i,1:3)-pose_and_covarianz(i-1,1:3);
        information_matrix(i-1,1)=1;
        %lb(i,1:3)=pose_and_covarianz(i,1:3)-0.01;
        %ub(i,1:3)=pose_and_covarianz(i,1:3)+0.01;
    end
    
end


%z_head_0=pose_and_covarianz(2:number_of_points,1:3);%zeros(3,size(z,1))';
beginning_v=current_v(1,1:3);
imu_bias_0=[0 0 0]
error=imu_bias_estimation(imu_bias_0,beginning_v,beginning_pos,end_pos,a_0)
%%

%lb = ones(3,number_of_points)'*-0.005;
%ub = ones(3,number_of_points)'*0.005;
hold on
%x = lsqnonlin(@(z_head) F_to_be_minimized(z_head,covarianz,z) ,z_head_0,lb,ub);
for j =1:1000
    imu_bias_updated = fmincon(@(imu_bias) imu_bias_estimation(imu_bias,beginning_v,beginning_pos,end_pos,a_0) ,imu_bias_0);%,lb,ub);
    error=imu_bias_estimation(imu_bias_updated,beginning_v,beginning_pos,end_pos,a_0)
    
    
    v(1,1:3)=beginning_v+(a_0(1,1:3)+imu_bias_updated)*(a_0(2,4)-a_0(1,4));
    for i = 2:(size(a_0,1)-1)
        v(i,1:3)=v(i-1,1:3)+(a_0(i,1:3)+imu_bias_updated)*(a_0(i+1,4)-a_0(i,4));
    end

    %forward_simulate_given_v:
    current_pos(1,1:3)=beginning_pos;
    for i =1:size(v,1)
        current_pos(i+1,1:3)=current_pos(i,1:3)+v(i,1:3)*(a_0(i+1,4)-a_0(i,4));%time(i+1)-time(i) 
    end
    
    imu_bias_0=imu_bias_updated
    beginning_v=v(end,1:3)
    
    scatter3(current_pos(:,1),current_pos(:,2),current_pos(:,3),[],pose_and_covarianz(2:(number_of_points+1),5))
    
end
axis equal

%%
% pose_robot(1,1:5)=beginning_pos;
% for i = 2:(size(x,1)+1)
%     pose_robot(i,1:5)=[pose_robot(i-1,1:3)+x(i-1,1:3) 0 pose_and_covarianz(i,5)];
% end

%hold on 

%scatter3(pose_and_covarianz(1:number_of_points,1),pose_and_covarianz(1:number_of_points,2),pose_and_covarianz(1:number_of_points,3),[],pose_and_covarianz(1:number_of_points,5))
%axis equal
%scatter3(pose_robot(:,1),pose_robot(:,2),pose_robot(:,3),[],pose_robot(:,5))


