close all
clear
clc

data=dlmread("TEST02.TXT");
t=data(:,1);
x=data(:,2);
y=data(:,3);
z=data(:,4);
plot(t,z,'r')

title('GPS Speed');
xlabel('Time (sec)');
ylabel('Speed (Knots)');
legend('Speed')
grid on