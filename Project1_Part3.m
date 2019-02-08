%ME 390 Project Script
%Implementation of ABS Control

clc %Clear command window
clear all;  % Clear all variables
close all;  % Close all figures

global L B R_w L1 L2 h Jrl toggle_ABS slip_ref Pbmax tau_lag tau_b G
global m g mw Iz Cf Cr steer_angle alpha_maxf alpha_maxr
global Fyfd Xtdon Xtdoff V_x Power_rated Gratio
global mu ifl ifr irl irr v_min fr break_on 

%Define Ground Vehicle Data for 2004 Envoy XL
EnvoyXL



Fyfd = 0; %Distrubance force
t_disturb = 100; %time to disturb car

Toggle_Anim = 'off';
%Not steering
steer_angle = -0*pi/180;
break_on = 60; %time to turn on ABS braking


% Hydraulic model
tau_lag = .01;      % Time constant for hydraulic brake model
Pbmax = 10.34e6;    % Maximum pressure for brake system, Pa or N/m^2, 10.34e6 Pa = 1500 psi
tau_b = .1;          % Gain between drive pressure rate of change and brake pressure
                    % This value represents a localized pressure buildup in the
                    % cylinder, and is likely related to compressibility and losses
                    % (i.e., an effective time constant)
G = 10*1.96e-4;        % Brake gain, N-m/(Pa) = effective radius*mu*area of slave cylinder
                    % allows you to write: Torque = G*Pressure
                    % This is Kf in Matlab/Simulink model.  Kf = 1 lbf-ft/psi = 1.966e-4 N-m/Pa
                    % From Gillespie: 20 in-lbf/psi (see p. 75) - passenger vehicle
                    % (this would be 1.7 lbf-ft/psi)
                    % You can estimate your max braking torque:
                    % G*Pbmax = about 2000 N-m
                    
slip_ref = 0.2;     % ABS slip control setting

% Initial Conditions
vx0 = 0;%initial translational velocity, m/s
vy0 = 0;
Xt0 = -100;
Yt0 = 0;
psi0 = 0;
omegaz0 = 0;
omega_fl0 =0;%vx0/R_w;%0;
omega_fr0 =0;%vx0/R_w;%0;
omega_rl0 =0;%vx0/R_w;%0;
omega_rr0 =0;%vx0/R_w;%0;
Pb_fl0 =0; % initial brake pressure
Plag_fl0 =0;
Pb_fr0 =0; % initial brake pressure
Plag_fr0 =0;
Pb_rl0 =0; % initial brake pressure
Plag_rl0 =0;
Pb_rr0 =0; % initial brake pressure
Plag_rr0 =0;

% q0 = [vx0;Xt0;omega_fl0; omega_fr0; omega_rl0; omega_rr0; ...
%     Pb_fl0; Plag_fl0; Pb_fr0; Plag_fr0; Pb_rl0; Plag_rl0; Pb_rr0; Plag_rr0];

q0 = [vx0;vy0;omegaz0;Xt0;Yt0;psi0;omega_fl0; omega_fr0; omega_rl0; omega_rr0; ...
    Pb_fl0; Plag_fl0; Pb_fr0; Plag_fr0; Pb_rl0; Plag_rl0; Pb_rr0; Plag_rr0];


% Parameters related to simulations
to = 0;
tf = 60;  % Simulation time, s
dt = 0.1;
n1 = floor(tf/dt);
tf2 = 68;
dt = 0.001;
n2 = floor((tf2-tf)/dt);  % Number of iterations

% Run Simulations
display('running sim 1')
[t1,q1]=rk4fixed('bicycle_model_ABS',[to tf],q0,n1);
display('running sim 2')
[t2,q2]=rk4fixed('bicycle_model_ABS',[tf tf+8],q1(end,:)',n2);

% Extract Results

xdot = [];
y = [];
for j = 1:length(t1)
    [X,Y] = bicycle_model_ABS(t1(j),q1(j,:));
    xdot(j,:) = X';
    y(j,:) = Y;
end
for i = 1:length(t2)
    [X,Y] = bicycle_model_ABS(t2(i),q2(i,:));
    xdot(i+j,:) = X';
    y(i+j,:) = Y;
end

T = [t1; t2];


% Plot results

subplot(2,2,1)
hold on
vx = [q1(:,1); q2(:,1)];
plot(T,vx)
Ofl = [q1(:,7); q2(:,7)];
Ofr = [q1(:,8); q2(:,8)];
Orl = [q1(:,9); q2(:,9)];
Orr = [q1(:,10); q2(:,10)];
omega_front = (Ofl+Ofr)/2;
omega_rear = (Orl+Orr)/2;

plot(T,omega_front.*R_w)
plot(T,omega_rear.*R_w)
legend('Vehicle','Front wheel','Rear Wheel')
xlabel('Time [s]')
ylabel('Velocity [m/s]')
title('Vehicle speed and wheel speed')
grid on
hold off
subplot(2,2,2)   
s_front = (y(:,1)+y(:,2))/2;
s_rear = (y(:,3)+y(:,4))/2;
hold on
plot(T,s_front)
plot(T,s_rear)
hold off
xlabel('Time [s]')
ylabel('Slip')
title('slip')
legend('Front','Rear')
grid on
subplot(2,2,3)   
title('Pressure')
hold on
plot(T,y(:,5))
plot(T,y(:,6))
xlabel('Time [s]')
ylabel('Pressure')
legend('Pb_front','Pb_rear')
grid on
hold off
subplot(2,2,4)   

dist = sqrt(q2(:,4).^2 + q2(:,5).^2);
dist = dist-dist(1);

plot(t2,dist)
title('Stopping Distance Plot')
xlabel('Time [s]')
ylabel('Stoping Distance [m]')
grid on