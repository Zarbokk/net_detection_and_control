function [F] = velocity_function_slam(v,information_matrix,v_beginning,beginning_pos,end_pos,time)
%TMP Summary of this function goes here
%   Detailed explanation goes here

    %virtual_meas=x-x_1
    
    %forward_simulate_given_v:
    current_pos(1,1:3)=beginning_pos;
    for i =1:size(v,1)
        current_pos(i+1,1:3)=current_pos(i,1:3)+v(i)*(0.02);%time(i+1)-time(i)
        
    end
    
    e_pos=norm(end_pos-current_pos(size(v,1)+1,1:3));%
    
    e_v=(v_beginning-v);
    for i = 1:size(e_v,1)
      F(i)=  e_v(i,:)*information_matrix(i)*e_v(i,:)';
    end
    for i = 1:(size(v,1)-1)
        v_diff(i)=norm(v(i+1)-v(i));
    end
    
    
    
    F=sum(F)+e_pos*10000000+sum(v_diff)*1;
end

