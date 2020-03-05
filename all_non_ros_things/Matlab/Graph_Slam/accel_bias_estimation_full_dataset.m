clc
clear
load('pose_imu_data.mat')

v_current=[0 0 0];
imu_bias_current=[0 0 0];
n=1
for i = 1:(size(ekf_pose_interest,1)-1)
    clear time imu_data_current current_imu_accel_ned v current_pos
    imu_data_current=imu_data_interest(find(imu_data_interest(:,end)>ekf_pose_interest(i,4) & imu_data_interest(:,end)<ekf_pose_interest(i+1,4)),:);
    number_of_points=size(imu_data_current,1);
    if number_of_points>0
    beginning_pos=ekf_pose_interest(i,1:3);
    end_pos=ekf_pose_interest(i+1,1:3);
    time(1)=imu_data_current(1,10)-ekf_pose_interest(i,4);
    for j = 1:(number_of_points-1)
        time(j)=imu_data_current(j+1,10)-imu_data_current(j,10);
    end
    time(number_of_points)=ekf_pose_interest(i+1,4)-imu_data_current(end,10);
    
    for j = 1:size(imu_data_current,1)
        rotation_matrix_current=euler2R(imu_data_current(j,1:3));
        current_imu_accel_ned(j,1:3)=(rotation_matrix_current*imu_data_current(j,4:6)'-[0 0 9.82]')';
    end
    
    imu_bias_current = fmincon(@(imu_bias) imu_bias_estimation(imu_bias,v_current,beginning_pos,end_pos,current_imu_accel_ned,time),imu_bias_current);%,lb,ub);
    
    
    
    v(1,1:3)=v_current+(current_imu_accel_ned(1,1:3)+imu_bias_current)*(time(1));
    for j = 2:(size(current_imu_accel_ned,1)-1)
        v(j,1:3)=v(j-1,1:3)+(current_imu_accel_ned(j,1:3)+imu_bias_current)*(time(j));
    end

    %forward_simulate_given_v:
    current_pos(1,1:3)=beginning_pos;
    for j =1:size(v,1)
        current_pos(j+1,1:3)=current_pos(j,1:3)+v(j,1:3)*(time(j));%time(i+1)-time(i) 
    end

    v_current=(rotation_matrix_current*[norm(v(end,1:3))*(1-1)+1*norm((ekf_pose_interest(i+1,1:3)-ekf_pose_interest(i,1:3))/(ekf_pose_interest(i+1,4)-ekf_pose_interest(i,4))) 0 0]')';
    
    plotting_color(n)=1;
    for j=1:size(current_pos,1)
        pose_boat(n,1:3) = current_pos(j,1:3);
        plotting_color(n+1)=plotting_color(n)+1;
        n=n+1;
    end
    else
        pose_boat(n,1:3) = end_pos;
        plotting_color(n)=1;
        n=n+1;
    end

end

%scatter3(pose_boat(:,1),pose_boat(:,2),pose_boat(:,3),[],plotting_color(1:(end-1)))%pose_and_covarianz(:,5))
scatter(pose_boat(:,1),pose_boat(:,2),[],plotting_color(1:(end-1)))%pose_and_covarianz(:,5))

axis equal
%plot(pose_boat(:,1),pose_boat(:,2))
