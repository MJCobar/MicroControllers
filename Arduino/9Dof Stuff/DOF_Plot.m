close all
clear
clc

data=dlmread("TEST.TXT");
t=data(:,1);
x=data(:,2);
y=data(:,3);
z=data(:,4);
plot(t,x,'r')
hold on
plot(t,y,'g')
plot(t,z,'b')

title("9Dof Data");
xlabel("Time (sec)");
ylabel("Angle (degree)");
legend('Yaw','Pitch','Roll')
grid on