%% clear
clc;clear;close all;
%% Parameters
Km = 0.869; % Constant of motor torque
Ke = 0.083; % Constant of the motor's back-EMF
L = 1.7; % Length of the pendulum
R = 0.2; % Wheel radius
Rm = 1; % Resistance of motor
Mp = 85; % Mass of the pendulum
Ip = 68.98; % Inertia of the pendulum
Mw = 3.5; % Mass of the wheel
Iw = 0.07; % Inertia of the wheel
g = 9.81; % Gravity
slope = 0.5*pi; % Surface slope
beta = 2*Mw + ((2*Iw) / (R*R)) + Mp;
alpha = Ip*beta + 2*Mp*L*L*((Mw*Iw) / (R*R));
%% StateSpace
A = [0 1 0 0;
   0 (2*Km*Ke*(Mp*L*R - Ip - Mp*L^2))/(Rm*(R^2)*alpha) ((Mp^2)*g*L^2)/alpha 0;
   0 0 0 1;
   0 (2*Km*Ke*(R*beta - Mp*L))/(Rm*(R^2)*alpha) (Mp*g*L*beta)/alpha 0];
B = [0;
    (2*Km*(Ip + Mp*L^2 - Mp*L*R))/(Rm*R*alpha);
    0;
     (2*Km*(Mp*L - R*beta))/(Rm*R*alpha)];
 C = [0 1 0 0; 0 0 1 0]; % outputs( xdot theta)

 %% Transfer Function
 [num,den] = ss2tf(A,B,C(1,1:end),0);
 tf_xdot_input = tf(num,den);
 [num,den] = ss2tf(A,B,C(2,1:end),0);
 tf_theta_input = tf(num,den);
 
 %% controller - theoric
Gc_theta = pid(834,1112,139);
sys_theta = feedback(Gc_theta * tf_theta_input,1);

figure(1);
subplot(2,2,1);
rlocus(tf_theta_input);
title("root locus - without controller");
subplot(2,2,2);
rlocus(tf_theta_input*Gc_theta);
title("root locus - with controller");
subplot(2,2,3);
step(tf_theta_input);
title("step response - open loop");
subplot(2,2,4);
step(sys_theta,10);
title("step response - with controller");
stepinfo(sys_theta)

%% controller - tune
Gc_theta_tune = pid(930,1650,300);
sys_theta_tune = feedback(Gc_theta_tune * tf_theta_input,1);
figure(2);
subplot(2,1,1);
step(sys_theta,3);
title("step response - theoric");
subplot(2,1,2);
step(sys_theta_tune,3);
title("step response - tuned");
stepinfo(sys_theta_tune)

 
 
 
 
 
 
 
 
 
 

