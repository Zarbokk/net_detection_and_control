%% calibration distance general

clc
clear

point_clouds = readmatrix("point_cloud.csv");
gantry_pos = readmatrix("gantry_pos.csv");
%%
%which_cloud_1 = [1450,1750,1850];
which_cloud_1 = [1550,1450,1850];
f1=figure(1);
tiledlayout(1,3)

for i=1:3
    
    which_cloud=which_cloud_1(i);
    current_pos = gantry_pos(gantry_pos(:,4)==which_cloud,:);
    current_pos(2) = current_pos(2)+0.2;

    y_area=0.15;

    current_cloud = point_clouds(point_clouds(:,8)==which_cloud,:);

    current_cloud = current_cloud(current_cloud(:,1)> -1 & current_cloud(:,1)< 1,:);
    current_cloud = current_cloud(current_cloud(:,2)> -y_area & current_cloud(:,2)< y_area,:);
    current_cloud = current_cloud(current_cloud(:,3)> 0 & current_cloud(:,3)< 4,:);
    current_cloud = current_cloud(current_cloud(:,2)> -5,:);

    for i = 1:size(current_cloud,1)
        change = -0.5 * current_cloud(i, 1)^2 + 0.1769 * current_cloud(i, 2)^2-current_cloud(i,3)*0.3;
        %change = -0.0 * current_cloud(i, 1)^2 + 0.0 * current_cloud(i, 2)^2;
        current_cloud(i,3)=current_cloud(i,3)-change;
        current_cloud(i,1)=current_cloud(i,1);
    end
    
    nexttile
    hold on
    set(gcf,'Position',[100 100 500 200])

    %plot3(current_cloud(:,1),current_cloud(:,2),current_cloud(:,3), 'o')
    plot(current_cloud(:,1)+current_pos(2),current_cloud(:,3)+current_pos(1), '.')


    %zlabel('Z')
    %axis equal
    plot(current_pos(2),current_pos(1), 'o')



    p0=[3.68;-0.055-0.035];
    p1=[3.68-0.43;-0.055-0.035];
    p2=[3.68+0.04;-0.055+0.64];
    p3=[3.68+0.04;-0.055+0.64+0.04];
    p4=[3.68-0.475;-0.055+1.41];
    p5=[3.68;-0.055+1.41];
    ground_truth=[p0(1) p1(1) p2(1) p3(1) p4(1) p5(1);p0(2) p1(2) p2(2) p3(2) p4(2) p5(2)]';

    plot(ground_truth(:,2),ground_truth(:,1))
    angle=65/180*pi/2;
    
    FOV_x=[current_pos(2)+sin(angle);current_pos(2);current_pos(2)-sin(angle)];
    FOV_y=[current_pos(1)+cos(angle);current_pos(1);current_pos(1)+cos(angle)];
    plot(FOV_x,FOV_y)
    %axis equal
    %
    %set(gca, 'YDir','reverse')
    xlabel('x-axis in m','interpreter','latex')
    ylabel('y-axis in m','interpreter','latex')

    axis([-0.5 1.7 0.1 4])
    grid on
end


%%

print(f1,'figure_exp_1.pdf','-dpdf','-r0')
system('pdfcrop figure_exp_1.pdf figure_exp_1.pdf');



