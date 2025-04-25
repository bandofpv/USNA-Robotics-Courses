% Parameters for Pendulum Cart Exercise
% EW306H Advanced Control Engineering
% Spring 2025
clear;
close all;

% System Parameters
M=0.261;                %Mass of Cart (kg)
m=0.125;                %Mass of pendulum (kg)
l=0.337;                %Length from pivot to pendulum's center of mass (m)
g = 9.81;               %Gravitational Constant (m/s^2)

% Initial Conditions
x_0 = 0.0;              %Initial Position of Cart (m)
x_dot_0 = 0.0;          %Initial Velocity of Cart (m/s)
theta_0 = 0.0;          %Initial Angle of Pendulum (rad)
theta_dot_0 = 0.0;      %Initial Angular Velocity of Pendulum (rad/s)

% Control Gains 
K = [-1.4584 -9.0946 -1.1371 -1.0944];
CG = -1.0944;
r = 0.1;              %Desired Position of Cart (m)

% Simulation 
EndTime = 6;          %End time for simulation
sim('PendulumCart_Block', 0:0.1:EndTime)

F_v = x(end);
OS = (max(x)-F_v)/F_v;
u_0 = u(1);
tetha_max = max(theta);

% Plots
figure(1); clf
plot(tout, x);
yline(F_v*0.98, 'color', 'green');
yline(F_v*1.02, 'color', 'green');
title("Cart Position vs. Time");
xlabel("Time (sec)");
ylabel("Cart Position (m)");
legend("Cart Position", "F_v +/- 2%", "", 'location', 'southeast')

figure(2); clf
plot(tout, theta);
title("Pendulum Angle vs. Time");
xlabel("Time (sec)");
ylabel("Pendulum Angle (rad)");

figure(3); clf
plot(tout, u)
title("Control Input vs. Time");
xlabel("Time (sec)");
ylabel("Control Input (N)")

% Animation
% PendulumCart_Anim(x,theta,tout,l) 