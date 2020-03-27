clc
clear
load('pose_imu_data.mat')

v_0=[0 0 0];
imu_bias_0(1,1:3)=[0 0 0];
%imu_bias_0=[0 0 0];
%n=1
for i = 1:(size(ekf_pose_interest,1)-1)
    %clear time imu_data_current current_imu_accel_ned v current_pos
    imu_data_current=imu_data_interest(find(imu_data_interest(:,end)>ekf_pose_interest(i,4) & imu_data_interest(:,end)<ekf_pose_interest(i+1,4)),:);
    number_of_points(i)=size(imu_data_current,1);
    
    beginning_pos(i,1:3)=ekf_pose_interest(i,1:3);
    end_pos(i,1:3)=ekf_pose_interest(i+1,1:3);
    if number_of_points(i)>0
        time(i,1)=imu_data_current(1,10)-ekf_pose_interest(i,4);
        for j = 1:(number_of_points(i)-1)
            time(i,j)=imu_data_current(j+1,10)-imu_data_current(j,10);
        end
        time(i,number_of_points(i))=ekf_pose_interest(i+1,4)-imu_data_current(end,10);

        for j = 1:number_of_points(i)
            rotation_matrix_current(i,j,1:3,1:3)=euler2R(imu_data_current(j,1:3));
            current_imu_accel_body(i,j,1:3)=[imu_data_current(j,4:6)];
            imu_bias_0(sum(number_of_points(1:(i-1)))+1+j,1:3)=[0 0 0];
        end
        
        
    else
        time(i,1)=ekf_pose_interest(i+1,4)-ekf_pose_interest(i,4);
        rotation_matrix_current(i,1:3,1:3)=[0 0 0;0 0 0;0 0 0];
        current_imu_accel_body(i,1,1:3)=[0 0 0];
    end
end


%%
options.Algorithm = 'sqp';
error_before=imu_bias_estimation(imu_bias_0,v_0,beginning_pos,end_pos,current_imu_accel_body,rotation_matrix_current,number_of_points,time)
imu_bias_with_correction_current = fmincon(@(imu_bias) imu_bias_estimation(imu_bias,v_0,beginning_pos,end_pos,current_imu_accel_body,rotation_matrix_current,number_of_points,time),imu_bias_0,[],[],[],[],0,[],[],options);%,lb,ub);
error_after=imu_bias_estimation(imu_bias_with_correction_current,v_0,beginning_pos,end_pos,current_imu_accel_body,rotation_matrix_current,number_of_points,time)


%%
n=1;
pos_robot(n,1:3)=beginning_pos(1,1:3);
n=n+1;
p=1;
current_v=[0 0 0];
for i = 1:size(time,1)
        if number_of_points(i)>0
            for j = 1:number_of_points(i)
                current_v=current_v+(imu_bias_with_correction_current(n-p,1:3)+(reshape(rotation_matrix_current(i,j,:,:),3,3)*(reshape(current_imu_accel_body(i,j,1:3),1,3)+imu_bias_with_correction_current(1,1:3))'-[0 0 9.81]')')*(time(i,j));
                pos_robot(n,1:3)=pos_robot(n-1,1:3)+current_v*(time(i,j));
                n=n+1;
            end
        else
            pos_robot(n,1:3)=pos_robot(n-1,1:3)+current_v*(time(i,1));
            p=p+1;
            n=n+1;
        end
        e(i)=norm(pos_robot(n-1,1:3)-end_pos(i,1:3));
end
hold on 
plot(pos_robot(:,1),pos_robot(:,2))
plot(beginning_pos(:,1),beginning_pos(:,2),'-o')
%plot(end_pos(:,1),end_pos(:,2),'-o')
%%


%         v(1,1:3)=v_current+(current_imu_accel_ned(1,1:3)+imu_bias_current)*(time(1));
%         for j = 2:(size(current_imu_accel_ned,1)-1)
%             v(j,1:3)=v(j-1,1:3)+(current_imu_accel_ned(j,1:3)+imu_bias_current)*(time(j));
%         end
% 
%         %forward_simulate_given_v:
%         current_pos(1,1:3)=beginning_pos;
%         for j =1:size(v,1)
%             current_pos(j+1,1:3)=current_pos(j,1:3)+v(j,1:3)*(time(j));%time(i+1)-time(i) 
%         end
% 
%         v_current=(rotation_matrix_current*[norm(v(end,1:3))*(1-1)+1*norm((ekf_pose_interest(i+1,1:3)-ekf_pose_interest(i,1:3))/(ekf_pose_interest(i+1,4)-ekf_pose_interest(i,4))) 0 0]')';
% 
%         plotting_color(n)=1;
%         for j=1:size(current_pos,1)
%             pose_boat(n,1:3) = current_pos(j,1:3);
%             plotting_color(n+1)=plotting_color(n)+1;
%             n=n+1;
%     end
%     else
%         pose_boat(n,1:3) = end_pos;
%         plotting_color(n)=1;
%         n=n+1;
%     end
% 
% end

%scatter3(pose_boat(:,1),pose_boat(:,2),pose_boat(:,3),[],plotting_color(1:(end-1)))%pose_and_covarianz(:,5))
scatter(pose_boat(:,1),pose_boat(:,2),[],plotting_color(1:(end-1)))%pose_and_covarianz(:,5))

axis equal
%plot(pose_boat(:,1),pose_boat(:,2))
