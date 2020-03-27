function [F] = tmp(x,x_pos,y_pos,z_pos,d)
%TMP Summary of this function goes here
%   Detailed explanation goes here
    F = zeros(size(x_pos,2),1);
    for i = 1:size(x_pos,2)
        F(i)=sqrt((x(1)-x_pos(i)).^2+(x(2)-y_pos(i)).^2+(x(3)-z_pos(i)).^2)-d(i);
    end
    for i= 1:size(x_pos,2)
        
        r_dist = sqrt((x(1)-x_pos(i)).^2+(x(2)-y_pos(i)).^2+(x(3)-z_pos(i)).^2);

        h_jac_x1 = 0.5 * 2.0 * (x(1)-x_pos(i)) / r_dist;

        h_jac_x2 = 0.5 * 2.0 * (x(2)-y_pos(i)) / r_dist;

        h_jac_x3 = 0.5 * 2.0 * (x(3)-z_pos(i)) / r_dist;

        J(i,1:3) = [h_jac_x1, h_jac_x2, h_jac_x3];
    end
    
end

