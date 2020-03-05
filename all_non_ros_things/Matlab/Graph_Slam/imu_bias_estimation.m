function [F] = imu_bias_estimation(imu_bias,beginning_v,beginning_pos,end_pos,a_0,time)
%TMP Summary of this function goes here
%   Detailed explanation goes here

    %virtual_meas=x-x_1
    %forward_simulate_given_a:
    v(1,1:3)=beginning_v+(a_0(1,1:3)+imu_bias)*(time(1));
    for i = 2:(size(a_0,1)-1)
        v(i,1:3)=v(i-1,1:3)+(a_0(i,1:3)+imu_bias)*(time(i));
    end
    
    %forward_simulate_given_v:
    current_pos(1,1:3)=beginning_pos;
    for i =1:size(v,1)
        current_pos(i+1,1:3)=current_pos(i,1:3)+v(i,1:3)*(time(i));%time(i+1)-time(i) 
    end
    
    e_pos=norm(end_pos-current_pos(size(v,1)+1,1:3));%

    F=e_pos;
end

