% Implements the partial linearizing scheme for 4W DD mobile robot as
% described in the paper "Trajectory Tracking Control of a Four-wheel 
% Differentially Driven Mobile Robot" by Luca Caracciolo, Alessandro 
% De Luca, Stefano Iannitti, 1999

% DOI: https://doi.org/10.1109/ROBOT.1999.773994

%https://www.youtube.com/watch?v=-71kForJ-oE

%% Clearing
clc;clear;close all;

%% Parameters 

m = 116; %Robot Mass, kg
I =  20; %Robot Inertia, kg.m2
M = [m 0 0; 0 m 0; 0 0 I];

a = 0.37; %front axle distance ,m
b = 0.55; %rear axle distance, m
t = 0.63/2; %wheel-to-centerline lateral distance, m
r = 0.2 ;   %tire radius, m
d0 = 0.18;  %offset point distance, m
g = 9.81;  %accel due to gravity, m/s2

%Actual soil parameters
% [X,Y,Z] = peaks(50);
% X=(X+3)*50/6;
% Y=(Y+3)*50/6;
% mu=Z*0.2/8+0.895;
% mesh(X,Y,mu);
% fr=Z*0.1/8+0.1;
% mesh(X,Y,fr);

mu_act = 0.88; %actual lateral friction coefficient
fr_act = 0.12; %actual coefficient of rolling resistance

%Estimated soil parameters (ideally slightly different from actual)
mu_est = 0.895; %lateral friction coefficient
fr_est = 0.1;   %coefficient of rolling resistance, assumed independent from velocity


%Limits
tau_max = 250; %Maximum allowed torque, in Nm
v_max = 2;     %Maximum longi speed, m/s

% Initial Conditions described in paper
% desired trajectory is zdl = t, zd2 = -0.002t3 + 0.1t2 + O.lt for t in [0,50] sec
% vehicle starts from the origin of frame F with a heading theta(0) = l0 deg", i.e., it is
% initially out of the desired trajectory. 
% Initial longitudinal and lateral (skidding) velocities are x_dot(0) = y_dot(0) = 0.5 m/s, 
% with angular velocity theta_dot(0) = -2.2 rad/s.

q0=[0;0;deg2rad(10)];
q0_dot = [[cos(q0(3)) -sin(q0(3));sin(q0(3)) cos(q0(3))]*[0.5;0.5];-2.2];
% q0=[0;0;0];
% q0_dot =[0;0;0];

% Track
Kp1=60;Kp2=60;
Kv1=40;Kv2=40;
Ka1=0;Ka2=0;

time = 0:0.1:50;
zd=[time;-0.002*time.^3+0.1*time.^2+0.1*time];
figure;plot(zd(1,:),zd(2,:),'Linewidth',1.5);title('Track');
xlabel('X');ylabel('Y');grid on;

%% Plot simulink results

figure;
subplot(2,1,1);
plot(out.q(:,1),out.q(:,2),'Linewidth',1.5);grid on;
ylabel('Y');xlabel('X');
subplot(2,1,2);
plot(out.t,rad2deg(out.q(:,3)),'Linewidth',1.5);grid on;ylabel('$\theta$ [deg]','Interpreter','Latex');

%% Debug Plots

figure;
subplot(2,1,1)
plot(out.t,out.tau(:,1),'-',out.t,out.taufbl(:,1),'--','Linewidth',1.5);
xlabel('t');ylabel('tau1');grid on;legend('ori','fbl');
subplot(2,1,2)
plot(out.t,out.tau(:,2),'-',out.t,out.taufbl(:,2),'--','Linewidth',1.5);
xlabel('t');ylabel('tau2');grid on;legend('ori','fbl');

% figure;
% subplot(2,1,1);
% plot(out.q(:,1),out.q(:,2),'Linewidth',1.5);grid on;
% ylabel('Y');xlabel('X');
% subplot(2,1,2);
% plot(out.t,rad2deg(out.q(:,3)),'Linewidth',1.5);grid on;ylabel('$\theta$ [deg]','Interpreter','Latex');

