close all;
clear;
clc;

data=dlmread('TEST02.TXT')
t=data(:,1);
x=data(:,2);
y=data(:,3);
z=data(:,4);
plot(t,x,'r');
axis([54,389,30.68,30.7]);

title('GPS Coordinates');
xlabel('Time (Sec)');
ylabel('Angle (Degree)');
legend('Latitude');
grid on