close all;
clear;
clc;

data=dlmread('TEST02.TXT')
t=data(:,1);
x=data(:,2);
y=data(:,3);
z=data(:,4);
plot(t,x,'r');
hold on
plot(t,y,'g');

title('GPS Coordinates');
xlabel('Time (Sec)');
ylabel('Angle (Degree)');
legend('Latitude','Longitude');
grid on