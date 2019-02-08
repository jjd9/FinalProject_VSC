%ME 390 Project Script
%Cruise Control

clc %Clear command window
clear all;  % Clear all variables
close all;  % Close all figures

global L B R_w L1 L2 h Jrl 
global m g mw Iz Cf Cr steer_angle alpha_maxf alpha_maxr
global Fyfd Xtdon Xtdoff V_x Power_rated Gratio cruise_on 
global mu ifl ifr irl irr v_min fr  break_on t_disturb
global K1 tau Kp Ki Kd taud Vssd cruise_control grade

%Define Ground Vehicle Data for 2004 Envoy XL
EnvoyXL


%Cruise Control settings
% Vssd =[20, 60, 30]; %Desired speeds, [m/s]
Vssd = 30;
cruise_control = 1;

%Gains tweaked to lower settling time and overshoot
Ki = 1;
Kd = 0.1;

grade = 60*pi/180;

Fyfd = 0; %Distrubance force
t_disturb = 50; %time to disturb car

Toggle_Anim = 'off';
%Not steering
steer_angle = -0*pi/180;
cruise_on = 2;
break_on = 0;


% Initial Conditions
vx0 = 0; %initial translational velocity, m/s
vy0 = 0;
omegaz0= 0;
Xt0 = 0;
Yt0 = 0;
psi0 = 0;
ie0 = 0;
z0 =0;
omega_fl0 =0;
omega_fr0 =0;
omega_rl0 =0;
omega_rr0 =0;

q0 = [vx0;vy0;omegaz0;Xt0;Yt0;psi0; ie0; z0; omega_fl0; omega_fr0; omega_rl0; omega_rr0];

% Parameters related to simulations
to = 0;
tf = 500;  % Simulation time, s
dt = 0.1;
n = floor(tf/dt);  % Number of iterations

To = to;
Tf = tf;

if strcmp(Toggle_Anim, 'on') == 1
    t = [to:dt:tf];
    
    %Set bounds of animation
    R_turn = 20*4;
    fig1 = figure(1);  % Figure set-up (fig1)
    axis([-R_turn 3*R_turn -R_turn/2 R_turn/2]); axis('square')
    hold on;
    
    % Acquire the configuration of vehicle for plot
    [xb, yb, xfrw, yfrw, xflw, yflw, xrrw, yrrw, xrlw, yrlw] = car_state(q0);
    
    % Plot vehicle and define plot id
    plot([-100,3*R_turn],[0,0],'--')
    plot([-100,3*R_turn],[5,5])
    plot([-100,3*R_turn],[-5,-5])
    plotqb = plot(xb, yb);  % Plot vehicle base
    plotqcg = plot(q0(4), q0(5),'.r'); % plot CG
    plotqflw = plot(xflw, yflw, 'r');  % Plot front left wheel
    plotqfrw = plot(xfrw, yfrw, 'r');  % Plot front right wheel
    plotqrlw = plot(xrlw, yrlw, 'r');  % Plot rear left wheel
    plotqrrw = plot(xrrw, yrrw, 'r');  % Plot rear right wheel
    plot_trail = plot(0,0,'o');
    
    q1 = q0;  % Set initial state to z1 for simulation

    xt = [];
    yt = [];
    % Beginning of simulation
    for i = 1:n+1
        to = t(i);
        tf = t(i)+dt;
        [t2,q2]=rk4fixed('bicycle_model_PID',[to tf],q1,2);
        t1 = t2(2);
        q1 = q2(2,:)';
        
        % Acquire the configuration of vehicle for plot
        [xb, yb, xfrw, yfrw, xflw, yflw, xrrw, yrrw, xrlw, yrlw] = car_state(q1);
        xt(i) = q1(4);
        yt(i) = q1(5);
        % Plot vehicle
        set(plotqb,'xdata',xb);
        set(plotqb,'ydata',yb);
        set(plot_trail,'xdata',xt)
        set(plot_trail,'ydata',yt)
        set(plotqcg,'xdata',q1(4),'ydata',q1(5));
        set(plotqflw,'xdata',xflw);
        set(plotqflw,'ydata',yflw);
        set(plotqfrw,'xdata',xfrw);
        set(plotqfrw,'ydata',yfrw);
        set(plotqrlw,'xdata',xrlw);
        set(plotqrlw,'ydata',yrlw);
        set(plotqrrw,'xdata',xrrw);
        set(plotqrrw,'ydata',yrrw);
        % drawnow
        pause(0.1);  % Pause by 0.2s for slower simulation
    end
    
end

[t,q]=rk4fixed('bicycle_model_PID',[To Tf],q0,n);

xdot = [];
y = [];
for i = 1:length(t)
    [X,Y] = bicycle_model_PID(t(i),q(i,:));
    xdot(i,:) = X';
    y(i,:) = Y;
end

labels = {'X veloctiy','Y veloctiy', ...
    'omegaz','X position', 'Y Position',...
    'psit'};
sz= size(q);
%         figure
for i = 1:6
    subplot(3,2,i)
    hold on
    plot(t,q(:,i))
    xlabel('Time [s]')
    ylabel(char(labels(i)))
    title(char(labels(i)))
    grid on
    
end
figure
subplot(1,2,1)
plot(t,y(:,10))
xlabel('Time [s]')
ylabel('Driving Torque [N*m]')
title('Driving Torque [N*m]')
grid on
subplot(1,2,2)
plot(t,y(:,11))
xlabel('Time [s]')
ylabel('System Error')
title('System Error')
grid on
Overshoot = max(q(:,1))-Vssd;