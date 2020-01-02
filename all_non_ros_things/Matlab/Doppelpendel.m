% Aus Aufgage A1 von FMKS. Winkel sind absolute Winkel bzgl. der -y Achse

clc;
clear all;
close all;

t = [0:0.01:1]; % Zeitvektor

Rb = [pi/4, pi/2, 0, 0]; % Randbedingungen

[Time,X] = ode45('Doppelpendel_DGL', t, Rb); % Lösen der DGL

l = 1;
S1 = zeros(length(X),1);

for i=1:length(X)
    S1(i) = ((981*cos(X(i, 2))*cos(conj(X(i, 1)))*cos(conj(X(i, 2))))/50 + (981*sin(X(i, 2))*cos(conj(X(i, 1)))*sin(conj(X(i, 2))))/50 + cos(X(i, 2))^2*cos(conj(X(i, 1)))*cos(conj(X(i, 2)))*X(i, 4)^2 + sin(X(i, 2))^2*sin(conj(X(i, 1)))*sin(conj(X(i, 2)))*X(i, 4)^2 + 2*sin(X(i, 1))*sin(X(i, 2))*sin(conj(X(i, 1)))*sin(conj(X(i, 2)))*X(i, 3)^2 + 2*cos(X(i, 1))*cos(X(i, 2))*cos(conj(X(i, 1)))*cos(conj(X(i, 2)))*X(i, 3)^2 + 2*cos(X(i, 1))*sin(X(i, 2))*cos(conj(X(i, 1)))*sin(conj(X(i, 2)))*X(i, 3)^2 + 2*cos(X(i, 2))*sin(X(i, 1))*cos(conj(X(i, 2)))*sin(conj(X(i, 1)))*X(i, 3)^2 + cos(X(i, 2))*sin(X(i, 2))*cos(conj(X(i, 1)))*sin(conj(X(i, 2)))*X(i, 4)^2 + cos(X(i, 2))*sin(X(i, 2))*cos(conj(X(i, 2)))*sin(conj(X(i, 1)))*X(i, 4)^2)/(cos(X(i, 1))*cos(X(i, 2))*cos(conj(X(i, 1)))*cos(conj(X(i, 2))) + 2*cos(X(i, 1))*sin(X(i, 2))*cos(conj(X(i, 1)))*sin(conj(X(i, 2))) - cos(X(i, 1))*sin(X(i, 2))*cos(conj(X(i, 2)))*sin(conj(X(i, 1))) - cos(X(i, 2))*sin(X(i, 1))*cos(conj(X(i, 1)))*sin(conj(X(i, 2))) + 2*cos(X(i, 2))*sin(X(i, 1))*cos(conj(X(i, 2)))*sin(conj(X(i, 1))) + sin(X(i, 1))*sin(X(i, 2))*sin(conj(X(i, 1)))*sin(conj(X(i, 2))))
       ((981*cos(X(i, 1))*cos(conj(X(i, 1)))*cos(conj(X(i, 2))))/100 + (981*sin(X(i, 1))*cos(conj(X(i, 1)))*sin(conj(X(i, 2))))/100 + cos(X(i, 1))^2*cos(conj(X(i, 1)))*cos(conj(X(i, 2)))*X(i, 3)^2 + sin(X(i, 1))^2*sin(conj(X(i, 1)))*sin(conj(X(i, 2)))*X(i, 3)^2 + sin(X(i, 1))*sin(X(i, 2))*sin(conj(X(i, 1)))*sin(conj(X(i, 2)))*X(i, 4)^2 + cos(X(i, 1))*cos(X(i, 2))*cos(conj(X(i, 1)))*cos(conj(X(i, 2)))*X(i, 4)^2 + cos(X(i, 1))*sin(X(i, 1))*cos(conj(X(i, 1)))*sin(conj(X(i, 2)))*X(i, 3)^2 + cos(X(i, 1))*sin(X(i, 1))*cos(conj(X(i, 2)))*sin(conj(X(i, 1)))*X(i, 3)^2 + cos(X(i, 1))*sin(X(i, 2))*cos(conj(X(i, 1)))*sin(conj(X(i, 2)))*X(i, 4)^2 + cos(X(i, 2))*sin(X(i, 1))*cos(conj(X(i, 2)))*sin(conj(X(i, 1)))*X(i, 4)^2)/(cos(X(i, 1))*cos(X(i, 2))*cos(conj(X(i, 1)))*cos(conj(X(i, 2))) + 2*cos(X(i, 1))*sin(X(i, 2))*cos(conj(X(i, 1)))*sin(conj(X(i, 2))) - cos(X(i, 1))*sin(X(i, 2))*cos(conj(X(i, 2)))*sin(conj(X(i, 1))) - cos(X(i, 2))*sin(X(i, 1))*cos(conj(X(i, 1)))*sin(conj(X(i, 2))) + 2*cos(X(i, 2))*sin(X(i, 1))*cos(conj(X(i, 2)))*sin(conj(X(i, 1))) + sin(X(i, 1))*sin(X(i, 2))*sin(conj(X(i, 1)))*sin(conj(X(i, 2))));
end;

x1 = l*sin(X(:,1));
y1 = -l*cos(X(:,1));
x2 = l*sin(X(:,1))+l*sin(X(:,2));
y2 = -l*cos(X(:,1))-l*cos(X(:,2));

figure(1)
plot(x1,y1,'r','LineWidth',2);
hold on
plot(x2,y2,'b','LineWidth',2);
title('Position')
grid on
xlabel('x')
ylabel('y')

figure(2)
plot(t,S1,'r','LineWidth',2);
title('Stabkraft')
grid on
xlabel('t')
ylabel('S1')