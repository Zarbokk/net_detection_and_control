%% calibration distance general

clc
clear

point_clouds = readmatrix("point_cloud.csv");
gantry_pos = readmatrix("gantry_pos.csv");
%% ground truth

p0=[3.68;-0.055-0.035];
p1=[3.68-0.43;-0.055-0.035];
p2=[3.68+0.04;-0.055+0.64];
p3=[3.68+0.04;-0.055+0.64+0.04];
p4=[3.68-0.475;-0.055+1.41];
p5=[3.68;-0.055+1.41];
ground_truth=[p0(1) p1(1) p2(1) p3(1) p4(1) p5(1);p0(2) p1(2) p2(2) p3(2) p4(2) p5(2)]';
gt_linspace = [linspace(ground_truth(1,1),ground_truth(2,1)),linspace(ground_truth(2,1),ground_truth(3,1)),linspace(ground_truth(3,1),ground_truth(4,1)),linspace(ground_truth(4,1),ground_truth(5,1)),linspace(ground_truth(5,1),ground_truth(6,1));linspace(ground_truth(1,2),ground_truth(2,2)),linspace(ground_truth(2,2),ground_truth(3,2)),linspace(ground_truth(3,2),ground_truth(4,2)),linspace(ground_truth(4,2),ground_truth(5,2)),linspace(ground_truth(5,2),ground_truth(6,2))]';


%%
%which_cloud_1 = [1450,1750,1850];
which_cloud_1 = [1550,1450,1850];
f1=figure(1);
tiledlayout(1,3)

for i=1:3
    
    which_cloud=which_cloud_1(i);
    current_pos = gantry_pos(gantry_pos(:,4)==which_cloud,:);
    current_pos(2) = current_pos(2)+0.1;

    y_area=0.15;

    current_cloud = point_clouds(point_clouds(:,8)==which_cloud,:);

    current_cloud = current_cloud(current_cloud(:,1)> -1 & current_cloud(:,1)< 1,:);
    current_cloud = current_cloud(current_cloud(:,2)> -y_area & current_cloud(:,2)< y_area,:);
    current_cloud = current_cloud(current_cloud(:,3)> 0 & current_cloud(:,3)< 4,:);
    current_cloud = current_cloud(current_cloud(:,2)> -5,:);

    for j = 1:size(current_cloud,1)
        current_cloud(j,1:3)=current_cloud(j,1:3)*1.3;
        rotation_point=[0 0 norm(current_cloud(j,1:3))]';

        alpha_x = -5/180*pi;
        if current_cloud(j,2)>0
            alpha_x=alpha_x*-1;
        end
        alpha_y=8/180*pi;
        if current_cloud(j,1)>0
            alpha_y=alpha_y*-1;
        end
        rotation_matrix_x = [1 0 0 ; 0 cos(alpha_x) -sin(alpha_x); 0 sin(alpha_x) cos(alpha_x)];
        rotation_matrix_y =[cos(alpha_y) 0 sin(alpha_y); 0 1 0;-sin(alpha_y) 0 cos(alpha_y)];
        rotation=rotation_matrix_y*rotation_matrix_x;
        %change = -0.5 * current_cloud(i, 1)^2 + 0.1769 * current_cloud(i, 2)^2-current_cloud(i,3)*0.3;
        %change = -0.0 * current_cloud(i, 1)^2 + 0.0 * current_cloud(i, 2)^2;
        %current_cloud(i,3)=current_cloud(i,3)-change;
        current_cloud(j,1:3)=(rotation*(current_cloud(j,1:3)'-rotation_point)+rotation_point)';
        if i==1
            error_1(j)=min(vecnorm((gt_linspace-[current_cloud(j,3)+current_pos(1) current_cloud(j,1)+current_pos(2)])'));
            gantry_pos_1 = current_pos;
        end
        if i==2
            error_2(j)=min(vecnorm((gt_linspace-[current_cloud(j,3)+current_pos(1) current_cloud(j,1)+current_pos(2)])'));
            gantry_pos_2 = current_pos;
        end
        if i==3
            error_3(j)=min(vecnorm((gt_linspace-[current_cloud(j,3)+current_pos(1) current_cloud(j,1)+current_pos(2)])'));
            gantry_pos_3 = current_pos;
        end
        
    end
    
    nexttile
    hold on
    set(gcf,'Position',[100 100 500 200])

    %plot3(current_cloud(:,1),current_cloud(:,2),current_cloud(:,3), 'o')
    plot(current_cloud(:,1)+current_pos(2),current_cloud(:,3)+current_pos(1), '.')


    %zlabel('Z')
    %axis equal
    plot(current_pos(2),current_pos(1), 'o')





    plot(ground_truth(:,2),ground_truth(:,1),'LineWidth',1.5)
    angle=70/180*pi/2;
    
    FOV_x=[current_pos(2)+sin(angle);current_pos(2);current_pos(2)-sin(angle)];
    FOV_y=[current_pos(1)+cos(angle);current_pos(1);current_pos(1)+cos(angle)];
    plot(FOV_x,FOV_y)
    %axis equal
    %
    %set(gca, 'YDir','reverse')
    xlabel('x-axis in m','interpreter','latex')
    ylabel('z-axis in m','interpreter','latex')

    axis([-0.5 1.7 0.1 4])
    grid on
end

%% median std mean calc

mean_c1 = mean(error_1)
std_c1 = std(error_1)
median_c1 = median(error_1)
gantry_pos_1(1:3)
mean_c2 = mean(error_2)
std_c2 = std(error_2)
median_c2 = median(error_2)
gantry_pos_2(1:3)
mean_c3 = mean(error_3)
std_c3 = std(error_3)
median_c3 = median(error_3)
gantry_pos_3(1:3)
print(f1,'figure_exp_1.pdf','-dpdf','-r0')
system('pdfcrop figure_exp_1.pdf figure_exp_1.pdf');



