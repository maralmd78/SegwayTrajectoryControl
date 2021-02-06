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
 C = [0 1 0 0; 0 0 1 0;1 0 0 0]; % outputs( xdot theta x )

 %% Transfer Function
 [num,den] = ss2tf(A,B,C(1,1:end),0);
 tf_xdot_input = tf(num,den);
 [num,den] = ss2tf(A,B,C(2,1:end),0);
 tf_theta_input = tf(num,den);
 [num,den] = ss2tf(A,B,C(3,1:end),0);
 tf_x_input = tf(num,den);
 
 
 %% theta controller - theoric 
Gc_theta = pid(834,1112,139);
sys_theta = feedback(Gc_theta * tf_theta_input,1);

figure(1);
subplot(2,2,1);
rlocus(tf_theta_input);
title("root locus theta - without controller");
subplot(2,2,2);
rlocus(tf_theta_input*Gc_theta);
title("root locus theta - with controller");
subplot(2,2,3);
step(tf_theta_input);
title("step response theta - open loop");
subplot(2,2,4);
step(sys_theta,10);
title("step response theta - with controller");
stepinfo(sys_theta)

%% theta controller - tune
Gc_theta_tune = pid(930,1650,300);
sys_theta_tune = feedback(Gc_theta_tune * tf_theta_input,1);
figure(2);
subplot(2,1,1);
step(sys_theta,3);
title("step response theta - theoric");
subplot(2,1,2);
step(sys_theta_tune,3);
title("step response theta - tuned");
stepinfo(sys_theta_tune)

%% speed(xdot) controller - theoric
% state feedback pole placement
K = acker(A,B,[-1+i -1-i 0 -15]);
Anew = A - B*K;
[num,den] = ss2tf(Anew,B,C(1,1:end),0);
tf_xdot_input_new = tf(num,den);
sys_xdot = tf_xdot_input_new/dcgain(tf_xdot_input_new);

figure(3);
subplot(2,2,1);
rlocus(tf_xdot_input);
title("root locus xdot - without controller");
subplot(2,2,2);
rlocus(sys_xdot);
title("root locus xdot - with controller");
subplot(2,2,3);
step(tf_xdot_input);
title("step response xdot - open loop");
subplot(2,2,4);
step(sys_xdot,10);
title("step response xdot - with controller");
stepinfo(sys_xdot)



 
 
 
 
 
 
 
 
 
 

