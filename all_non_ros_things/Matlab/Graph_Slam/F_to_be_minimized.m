function [F] = F_to_be_minimized(x,information_matrix,z,beginning_pos)
%TMP Summary of this function goes here
%   Detailed explanation goes here

    %virtual_meas=x-x_1
    
    for i = 1:size(x,1)
        if i ==1
            z_head(i,1:3)=x(i,:)-beginning_pos;
        else
            z_head(i,1:3)=x(i,:)-x(i-1,:);
        end
        
    end
    
    
    e=(z-z_head);
    for i = 1:size(e,1)
      F(i)=  e(i,:)*information_matrix(i,i)*e(i,:)';
    end
    F=sum(F);
end

