%ME 390 Project Script
%Acceleration, Braking, and Double-Lane Change Test Cases


clc %Clear command window
clear all;  % Clear all variables
close all;  % Close all figures


global L B R_w L1 L2 Jrl turn_time
global m g Iz Cf Cr steer_angle alpha_maxf alpha_maxr
global Fyfd V_x Power_rated Gratio
global mu ifl ifr irl irr v_min fr break_on 

%Define Ground Vehicle Data for 2004 Envoy XL
EnvoyXL

%Select Simulation to Run
Opt = {'Accelerate', 'Brake', 'Turn'};
Simulation = char(Opt(1));
Toggle_Anim = 'off'; %Turn animation on and off


Fyfd = 0;%Not using disturbance force yet
turn_time = 0;

%Predict vehicle operating in:


%% Acceleration and Braking

%Must include traction forces (so need wheel dynamics sa well as simple
%drivetrain model, like motor with gear)


if strcmp(Simulation, char(Opt(1))) == 1
    display('ACCELERATION SIMULATION')
    %% Show vehicle starting from rest and reaching steady state
    % Initial values, Start from 'rest'
    
    vx0 = 0;
    vy0 = 0;
    omegaz0 = 0;
    Xt0 = 0;
    Yt0 = 0;
    psit0 = 0;
    omega_fl0 = 0;
    omega_fr0 = 0;
    omega_rl0 = 0;
    omega_rr0 = 0;
    q0 = [vx0; vy0; omegaz0; Xt0; Yt0; psit0; omega_fl0; omega_fr0; omega_rl0; omega_rr0];
    
    
    % Parameters related to simulations
    to = 0;
    tf = 200;  % Simulation time, s
    dt = 0.1;
    n = floor(tf/dt);  % Number of iterations
    %Not steering Yet!
    steer_angle = -0*pi/180;

    %Assuming no breaking force applied
    ifl = 0; ifr = 0; irl = 0; irr = 0;
    break_on = 0;
    
    %Based on this first analysis, when the car starts from rest given some
    %initial wheel rotation, it will consistently reach a steady state
    %longitudinal speed of 66 [m/s]. This speed occurs after 50 seconds of acceleration, so we will start breaking at this time for the next analysis.
    
    
elseif strcmp(Simulation, char(Opt(2))) == 1
    display('BRAKING SIMULATION')
    %% Show vehicle starting from steady-state and braking to stop
    %Assuming all wheels locked out
    ifl = 1; ifr = 1; irl = 1; irr = 1;
    

    % Initial conditions for vehicle states
    vx0 = 0;
    vy0 = 0;
    omegaz0 = 0;
    Xt0 = -100;
    Yt0 = 0;
    psit0 = 0;
    omega_fl0 = 0;
    omega_fr0 = 0;
    omega_rl0 = 0;
    omega_rr0 = 0;
    q0 = [vx0; vy0; omegaz0; Xt0; Yt0; psit0; omega_fl0; omega_fr0; omega_rl0; omega_rr0];
    
    
    % Parameters related to simulations
    to = 0;
    tf = 100;  % Simulation time, s
    dt = 0.1;
    n = floor(tf/dt);  % Number of iterations
    %Not steering Yet!
    break_on = 50;
    steer_angle = -0*pi/180;
    
    %Locking out all four wheels seems to be the fastest way to stop the
    %vehicle. All the lockout scenarios result in severe vehicle
    %instability. Future sections will include the implementation of an ABS
    %system to provide increased stability while braking.
    
elseif strcmp(Simulation, char(Opt(3))) == 1
    display('TURNING SIMULATION')
    %% Turning
    %Open-loop lane double lane change maneuvers
    steer_angle = 5*pi/180;
    turn_time = 10;
    % Initial values, Start from 'rest'
    V_x = 5*L;
    vx0 = V_x;
    vy0 = 0;
    omegaz0 = 0;
    Xt0 = -50;
    Yt0 = -1.5;
    psit0 = 0;
    omega_fl0 = 0;
    omega_fr0 = 0;
    omega_rl0 = 0;
    omega_rr0 = 0;
    q0 = [vx0; vy0; omegaz0; Xt0; Yt0; psit0; omega_fl0; omega_fr0; omega_rl0; omega_rr0];
    
    
    
    Fyfd = 0;
    % Parameters related to simulations
    to = 0;
    tf = 20;  % Simulation time, s
    dt = 0.1;
    n = floor(tf/dt);  % Number of iterations
    
    %Assuming no breaking force applied
    ifl = 0; ifr = 0; irl = 0; irr = 0;
    
    %I wanted to the car to already be at a constant speed when it started
    %turning. So I opted to "mute" the driving torque and frictional effects
    %and simply have the car run at a constant velocity.
end


To = to;
Tf = tf;

if strcmp(Toggle_Anim, 'on') == 1
    t = [to:dt:tf];
    
    %Set bounds of animation
    R_turn = 80;
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
        [t2,q2]=rk4fixed('bicycle_model_braking',[to tf],q1,2);
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

[t,q]=rk4fixed('bicycle_model_braking',[To Tf],q0,n);
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

%Determine Braking Distance
if length(break_on) ~= 0
    index1 = find(t>break_on,1);
    index2 = index1 + find(q(index1:end,1)<5,1);
    x = q(index1:index2,4);
    y = q(index1:index2,5);
    dist = sqrt(x.^2 + y.^2);
    dist = cumsum(dist-dist(1));
    dist(end)
end
