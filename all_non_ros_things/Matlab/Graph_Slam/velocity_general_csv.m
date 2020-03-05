clc
clear
format long

ekf_pose_raw = readmatrix("april_tag_ekf.csv");%x y z yaw time

imu_data_raw = readmatrix("imu_data.csv");%roll pitch yaw x_accel y_accel z_accel x_a_velocity y_a_valocity z_a_velocity time

calibration_tank = readmatrix("calibration_tank.csv");%id x y z 1

tag_detections = readmatrix("tag_detections.csv");% x y z r p y id number timestamp
%% tag detections to x y z pose 
n=1
i=1
clear pose_boat
x0 = [1.5 1 0];
while i < size(tag_detections,1)-15
    clear x_pos y_pos z_pos d
    id = tag_detections(i,7);
    tag_pose = calibration_tank(find(calibration_tank(:,1)==id),2:4);
    d(1) = norm(tag_detections(i,1:3));
    x_pos(1) = tag_pose(1);
    y_pos(1) = tag_pose(2);
    z_pos(1) = tag_pose(3);
    k=1;
    while tag_detections(i+k,9) == tag_detections(i,9)
        id = tag_detections(i+k,7);
        tag_pose = calibration_tank(find(calibration_tank(:,1)==id),2:4);
        k=k+1;
        d(k) = norm(tag_detections(i+k-1,1:3));
        x_pos(k) = tag_pose(1);
        y_pos(k) = tag_pose(2);
        z_pos(k) = tag_pose(3);
    end
    if k>2
        [x,resnorm,residual,exitflag,output] = lsqnonlin(@(x) tmp(x,x_pos,y_pos,z_pos,d),x0);
        if x(1)<4 && x(2)<2.5 && x(3)<1.8 && x(3)>-0.4
            
        
            pose_boat(n,1:3) = x;
            pose_boat(n,4) = tag_detections(i,9);
            x0 = x;
            n=n+1;
        end
    end
    i=i+k
end
%plot3(pose_boat(:,1),pose_boat(:,2),pose_boat(:,3))
%plot(pose_boat(:,1),pose_boat(:,2))
%%
%ekf_pose_plot=ekf_pose_raw(1000:(size(ekf_pose_raw,1)-1000),:);
%plot3(ekf_pose_raw(:,2),ekf_pose_raw(:,1),-ekf_pose_raw(:,3))
%axis equal

%area of interest:
%ekf_pose_interest=ekf_pose_raw(5000:(size(ekf_pose_raw,1)-5500),:);
%ekf_pose_interest(:,5)=ekf_pose_interest(:,5)+0.5
%start_time=min(ekf_pose_interest(:,end))
%end_time=max(ekf_pose_interest(:,end))

ekf_pose_interest=pose_boat(1:(size(pose_boat,1)-100),:);
ekf_pose_interest(:,4)=ekf_pose_interest(:,4)+0.0
start_time=min(ekf_pose_interest(:,end))
end_time=max(ekf_pose_interest(:,end))


imu_data_interest=imu_data_raw(find(imu_data_raw(:,end)>start_time & imu_data_raw(:,end)<end_time),:);
imu_data_interest(:,4)=-imu_data_interest(:,4);



ekf_pose_interest=ekf_pose_interest(1:9:end,:);


%plot3(ekf_pose_interest(:,1),ekf_pose_interest(:,2),ekf_pose_interest(:,3),'.')
%axis equal
%%
k=1
n=1
gain=0.9;
clear pose_and_covarianz
%pose_and_covarianz= zeros(size(ekf_pose_interest,1)-18+size(imu_data_interest,1),5);
pose_and_covarianz(k,1)=ekf_pose_interest(1,1);
pose_and_covarianz(k,2)=ekf_pose_interest(1,2);
pose_and_covarianz(k,3)=ekf_pose_interest(1,3);
pose_and_covarianz(k,4)=ekf_pose_interest(1,4);
pose_and_covarianz(k,5)=1;
current_v_body(k)=0;
k=k+1;

for i = 2:(size(ekf_pose_interest,1)-1)

    if i >2
        
        current_v_body(k)=current_v_body(k-1)*(1-gain)+gain*norm((pose_and_covarianz(k-1,1:3)-pose_and_covarianz(k-size(imu_data_current,1)-2,1:3))/(pose_and_covarianz(k-1,4)-pose_and_covarianz(k-size(imu_data_current,1)-2,4)));
    end
    
    imu_data_current=imu_data_interest(find(imu_data_interest(:,end)>ekf_pose_interest(i,4) & imu_data_interest(:,end)<ekf_pose_interest(i+1,4)),:);
    for j = 1:size(imu_data_current,1)
        if j==1
            time=imu_data_current(j,10)-ekf_pose_interest(i,4);
        else
            time=imu_data_current(j,10)-imu_data_current(j-1,10);
        end
        
        rotation_matrix_current=euler2R(imu_data_current(j,1:3));
        current_imu_accel(n,1:4)=[imu_data_current(j,4:6)-(inv(rotation_matrix_current)*[0 0 9.81]')' imu_data_current(j,10)];
        current_v_body(k)=current_v_body(k-1)-current_imu_accel(n,1)*time;
        velocity_plot(n,1:2)=[current_v_body(k) imu_data_current(j,10)];
        n=n+1;
        current_integrated_diff=time*rotation_matrix_current*[current_v_body(k) 0 0]';
        pose_and_covarianz(k,1:3)=pose_and_covarianz(k-1,1:3)+current_integrated_diff';
        pose_and_covarianz(k,4)=imu_data_current(j,10);
        pose_and_covarianz(k,5)=j+1;
        k=k+1;
    end
    pose_and_covarianz(k,1)=ekf_pose_interest(i,1);
    pose_and_covarianz(k,2)=ekf_pose_interest(i,2);
    pose_and_covarianz(k,3)=ekf_pose_interest(i,3);
    pose_and_covarianz(k,4)=ekf_pose_interest(i,4);
    pose_and_covarianz(k,5)=1;
    current_v_body(k)=current_v_body(k-1)*(1-gain)+gain*norm((pose_and_covarianz(k,1:3)-pose_and_covarianz(k-size(imu_data_current,1)-1,1:3))/(pose_and_covarianz(k,4)-pose_and_covarianz(k-size(imu_data_current,1)-1,4)));

    %current_v_body(k)=current_v_body(k-1)*(1-gain)+gain*norm((pose_and_covarianz(k-1,1:3)-pose_and_covarianz(k-size(imu_data_current,1)-2,1:3))/(pose_and_covarianz(k-1,4)-pose_and_covarianz(k-size(imu_data_current,1)-2,4)));
    k=k+1;
    
end
% for i =1:(size(imu_data_interest,1)-1)
%     rotation_matrix_current=euler2R(imu_data_interest(i,1:3));
%     current_imu_accel_NED=imu_data_interest(i,4:6)*rotation_matrix_current-[0 0 9.81];
%     imu_data_interest(i,1:3)=current_imu_accel_NED;
% end
%scatter(pose_and_covarianz(:,1),pose_and_covarianz(:,2),[],pose_and_covarianz(:,5))
 current_v_body=current_v_body';

%plot(current_imu_accel(:,4),current_imu_accel(:,2))
figure
scatter3(pose_and_covarianz(:,1),pose_and_covarianz(:,2),pose_and_covarianz(:,3),[],1:size(pose_and_covarianz,1))%pose_and_covarianz(:,5))
%scatter(pose_and_covarianz(:,4),pose_and_covarianz(:,1),[],pose_and_covarianz(:,5))
figure
hold on
plot(current_imu_accel(:,4),current_imu_accel(:,1))
plot(velocity_plot(:,2),velocity_plot(:,1))

save("pose_and_covarianz.mat",'pose_and_covarianz','current_v_body')