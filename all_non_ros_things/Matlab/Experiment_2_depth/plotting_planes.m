clc
clear

gantry_pos = readmatrix("gantry_pos_planes.csv");
planes_array = readmatrix("planes_all.csv");

%%
k=10;
f1=figure(1);
hold on
set(gcf,'Position',[100 100 500 200])
for i = 1:round(size(planes_array,1)*0.24)
    if mod(i,k)==0
        
        size_planes=0.05;
        x_gantry=gantry_pos(i,1);
        y_gantry=gantry_pos(i,2);

        m_r=planes_array(i,1);
        b_r=planes_array(i,2);
        m_l=planes_array(i,3);
        b_l=planes_array(i,4);
        p1_r=[0,b_r*1.3];
        p2_r=[size_planes,b_r*1.3+m_r*size_planes*1.3+0.5*size_planes];
        p1_r=p1_r+[y_gantry,x_gantry];
        p2_r=p2_r+[y_gantry,x_gantry];


        p1_l=[0,b_l*1.3];
        p2_l=[-size_planes,b_l*1.3-m_l*size_planes*1.3+0.5*size_planes];

        p1_l=p1_l+[y_gantry,x_gantry];
        p2_l=p2_l+[y_gantry,x_gantry];

        %plot([p1_r(1),p2_r(1)],[p1_r(2),p2_r(2)],'Color','b')%plot R
        plot([p1_l(1),p2_l(1)],[p1_l(2),p2_l(2)],'Color',[1-i/size(planes_array,1)/0.4 i/size(planes_array,1)/0.4 0],'LineWidth',2)%plot L
        
        plot(y_gantry,x_gantry, 'o','Color',[1-i/size(planes_array,1)/0.4 i/size(planes_array,1)/0.4 0])
        i/size(planes_array,1)/0.3
    end
end
%axis equal
%%
p0=[3.68;-0.055-0.035];
p1=[3.68-0.43;-0.055-0.035];
p2=[3.68+0.04;-0.055+0.64];
p3=[3.68+0.04;-0.055+0.64+0.04];
p4=[3.68-0.475;-0.055+1.41];
p5=[3.68;-0.055+1.41];
ground_truth=[p0(1) p1(1) p2(1) p3(1) p4(1) p5(1);p0(2) p1(2) p2(2) p3(2) p4(2) p5(2)]';

plot(ground_truth(:,2),ground_truth(:,1))
xlabel('x-axis in m','interpreter','latex')
ylabel('y-axis in m','interpreter','latex')

axis([-0.5 2 1.4 4])
grid on

print(f1,'figure_exp_2.pdf','-dpdf','-r0')
system('pdfcrop figure_exp_2.pdf figure_exp_2.pdf');