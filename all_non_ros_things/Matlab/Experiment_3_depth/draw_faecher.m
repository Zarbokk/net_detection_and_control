%% calibration position
clc
clear
format long
%%
time_stamps=[1599222843.781669,1599222857.989608;
    1599222866.528917,1599222882.259391;
    1599222894.348379,1599222906.185498;
    1599222941.689980,1599222954.950993;
    1599222969.732952,1599222983.131891;
    1599222992.284111,1599223008.151541;
    1599223019.780525,1599223033.242515;
    1599223042.294840,1599223058.947424;
    1599223069.367228,1599223082.948902;
    1599223094.450844,1599223108.002693;
    1599223122.431643,1599223135.363267;
    1599223149.732606,1599223162.854540;
    1599223173.359179,1599223186.732487;
    1599223196.494896,1599223213.019039;
    1599223224.693135,1599223238.593397;
    1599223251.677011,1599223265.470150;
    1599223279.887390,1599223294.321489;
    1599223309.270513,1599223324.234727;
    1599223338.375718,1599223353.197082;
    1599223385.404218,1599223395.302166;
    1599223404.258286,1599223420.714423;
    1599223433.469060,1599223447.332247;
    1599223486.274681,1599223498.771572;
    1599223522.709513,1599223538.025319;
    1599223556.990082,1599223570.528079;
    1599223584.858871,1599223599.724073;
    1599223616.939046,1599223633.123497;
    1599223645.186537,1599223660.188242;
    1599223672.045125,1599223687.389847;
    1599223709.485145,1599223724.726645;
    
];



% time_stamps=[1599127551.495920,1599127564.527216;
% 1599127612.815502,1599127626.111898;
% 1599127638.483013,1599127651.451990;
% 1599127663.061970,1599127676.449089;
% 1599127687.374835,1599127699.937137;
% 1599127715.578851,1599127728.717323;
% 1599127739.677547,1599127755.291758;
% 1599127769.547426,1599127782.747005;
% 1599127805.522499,1599127818.593228;
% 1599127828.702832,1599127842.825556;
% 1599127854.924027,1599127867.905596;
% 1599127879.465959,1599127892.826417];

% time_stamps=[1599074686.759959,1599074707.375115;
% 1599074801.218490,1599074815.307040;
% 1599074845.431095,1599074865.243715;
% 1599074888.755146,1599074904.796674;
% 1599074925.080188,1599074940.520868;
% 1599074969.554764,1599074983.741780];
% time_stamps=[1599202299.646492,1599202312.173763;
%     1599202333.638608,1599202345.910315;
%     1599202355.538366,1599202368.566451;
%     1599202378.241065,1599202389.879604;
%     1599202402.720922,1599202413.113137;
%     1599202422.805110,1599202436.355062;
%     1599202447.936467,1599202460.554368;
%     1599202468.669086,1599202482.135968;
%     1599202495.404516,1599202509.198088;
%     1599202522.444578,1599202533.961478;
%     1599202545.032654,1599202557.221763];


pose_boat_complete = readmatrix("pose_px4_boat.csv");
pc_complete = readmatrix("point_cloud.csv");

%%

poses_to_keep = pose_boat_complete((pose_boat_complete(:,6)>time_stamps(4,1) &  pose_boat_complete(:,6)<time_stamps(4,2)),:);

hold on

grid on
plot([1.8 1.8],[0 4])
%set(gcf,'Position',[100 100 700 200])
axis([-0.5 2 0 4])
%%
for i = 1:size(poses_to_keep,1)
    current_time=poses_to_keep(i,6);
    current_pos=poses_to_keep(i,1:3);
    current_yaw=-poses_to_keep(i,4)-pi/2;
    current_pointcloud = pc_complete(pc_complete(:,5)==current_time,:);
    current_color=[1-i/size(poses_to_keep,1) i/size(poses_to_keep,1) 0];
    rotation_matrix_y =[cos(current_yaw) 0 sin(current_yaw); 0 1 0;-sin(current_yaw) 0 cos(current_yaw)];
    for j = 1:size(current_pointcloud,1)
        current_pointcloud(j,1:3)=(rotation_matrix_y*current_pointcloud(j,1:3)')';
        x=current_pos(1)+current_pointcloud(j,1);
        y=current_pos(2)+current_pointcloud(j,3);
        z=current_pos(3)-current_pointcloud(j,2);
        x_list(j)=x;
        y_list(j)=y;
        z_list(j)=z;
    end
    %plot3(y_list,x_list,z_list,".","Color",current_color)
    plot(poses_to_keep(i,2),poses_to_keep(i,1),".","Color",current_color)
    %plot(current_pointcloud(:,2),current_pointcloud(:,1),".")
end
%axis equal

%%
for i = 2:30
    poses_to_keep = pose_boat_complete(pose_boat_complete(:,6)>time_stamps(i,1) &  pose_boat_complete(:,6)<time_stamps(i,2),:);
    plot(poses_to_keep(:,2),poses_to_keep(:,1),".")
end
%%
%poses_to_keep = pose_boat_complete(pose_boat_complete(:,6)>time_stamps(12,1) &  pose_boat_complete(:,6)<time_stamps(12,2),:);
%plot(poses_to_keep(:,2),poses_to_keep(:,1),".")
plot([0.7 0.7],[0 4])
xlabel('y-axis in m','interpreter','latex')
ylabel('x-axis in m','interpreter','latex')
%system('pdfcrop tmp.pdf tmp.pdf');


%% test
% clc
% clear
% hold on 
% m=1
% b=0.5
% x=linspace(-1,1);
% plot(x,m*x+b)
% alpha=0.2;
% m_new=(sin(alpha)+cos(alpha)*m)/(cos(alpha)-sin(alpha)*m)
% b_new=b/(cos(alpha)-sin(alpha)*m)
% plot(x,m_new*x+b_new)





