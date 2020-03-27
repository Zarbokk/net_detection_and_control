function F = tmp2(x,x_pos,y_pos,z_pos,d)
%TMP Summary of this function goes here
%   Detailed explanation goes here
    F = zeros(size(x_pos,2),1);
    for i = 1:size(x_pos,2)
        F(i)=(sqrt((x(1)-x_pos(i)).^2+(x(2)-y_pos(i)).^2+(x(3)-z_pos(i)).^2)-d(i))^2;
    end
    
    F=sum(F);
end

