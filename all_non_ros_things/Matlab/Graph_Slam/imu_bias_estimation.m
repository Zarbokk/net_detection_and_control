function [F] = imu_bias_estimation(imu_bias,v_0,beginning_pos,end_pos,current_imu_accel_body,rotation_matrix_current,number_of_points,time)
%TMP Summary of this function goes here
%   Detailed explanation goes here
%imu_bias(1,1:3) imu bias before rotation  imu_bias(2:end,1:3) bias for
%every individual measurement
    %virtual_meas=x-x_1
    %forward_simulate_given_a:
    %imu_bias=imu_bias_with_correction_current;
    %imu_bias=imu_bias_0;
    n=1;
    pos_robot(n,1:3)=beginning_pos(1,1:3);
    n=n+1;
    p=1;
    current_v=v_0;
    for i = 1:size(time,1)
        if number_of_points(i)>0
            for j = 1:number_of_points(i)
                current_v=current_v+(imu_bias(n-p,1:3)+(reshape(rotation_matrix_current(i,j,:,:),3,3)*(reshape(current_imu_accel_body(i,j,1:3),1,3)+imu_bias(1,1:3))'-[0 0 9.81]')')*(time(i,j));
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
    
   

    F=sum(e)+sum(norm(imu_bias(:,2:end)));
end

