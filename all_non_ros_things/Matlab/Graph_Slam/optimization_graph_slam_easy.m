clc
clear
load('pose_and_covarianz.mat')
beginning_pos=pose_and_covarianz(1,:);
number_of_points=4000


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


z_head_0=pose_and_covarianz(2:number_of_points,1:3);%zeros(3,size(z,1))';




%lb = ones(3,number_of_points)'*-0.005;
%ub = ones(3,number_of_points)'*0.005;

%x = lsqnonlin(@(z_head) F_to_be_minimized(z_head,covarianz,z) ,z_head_0,lb,ub);

x = fmincon(@(x) F_to_be_minimized(x,diag(information_matrix),z,beginning_pos(1,1:3)) ,z_head_0);%,lb,ub);


% pose_robot(1,1:5)=beginning_pos;
% for i = 2:(size(x,1)+1)
%     pose_robot(i,1:5)=[pose_robot(i-1,1:3)+x(i-1,1:3) 0 pose_and_covarianz(i,5)];
% end

%hold on 

%scatter3(pose_and_covarianz(1:number_of_points,1),pose_and_covarianz(1:number_of_points,2),pose_and_covarianz(1:number_of_points,3),[],pose_and_covarianz(1:number_of_points,5))
%axis equal
%scatter3(pose_robot(:,1),pose_robot(:,2),pose_robot(:,3),[],pose_robot(:,5))

scatter3(x(:,1),x(:,2),x(:,3),[],pose_and_covarianz(2:number_of_points,5))
axis equal