function [xdot, y] = bicycle_model_PID(t,x)
% Note: this is a 2D vehicle model that has braking and lateral forces at
% four wheels: fl, fr, rl, rr
% It is assumed that both front tires have same Cf, same for rear
% This is a simulation of fully-locked braking.
global L B R_w L1 L2 h Jrl t_disturb drivetrain_damp
global m g mw Iz Cf Cr steer_angle alpha_maxf alpha_maxr
global Fyfd V_x Power_rated Gratio cruise_on 
global mu ifl ifr irl irr v_min fr break_on grade
global K1 tau Kp Ki Kd taud Vssd cruise_control 
stop =1;


%Parse input
vx = x(1); vy = x(2); omegaz = x(3);
Xt = x(4); Yt = x(5); psit = x(6);
ie = x(7);
z = x(8);
omega_fl = x(9); omega_fr = x(10); omega_rl = x(11); omega_rr = x(12); 

%Switch on cruise control at a specific time
if length(Vssd)>1
if t>= 50*(cruise_control)
    if cruise_control +1 <=3
        cruise_control = cruise_control + 1;
        display('setting shift')

    end
end
end
if t >= cruise_on        
    r = Vssd(cruise_control);
else
    r = 0;
end

% no braking
ifl = 0; ifr = 0; irl = 0; irr = 0; 

deltaf = 0;
deltar = 0; % no rear steer

% compute the velocities at the four wheels
vxfl = vx - B*omegaz/2; vyfl = vy + L1*omegaz;
vxfr = vx + B*omegaz/2; vyfr = vy + L1*omegaz;

vxrl = vx - B*omegaz/2;
vyrl = vy - L2*omegaz;
vxrr = vx + B*omegaz/2;
vyrr = vy - L2*omegaz;

% double lane change (nothing will happen when steer_angle is set to 0)
if (t<1) deltaf = 0; end;
if (t>=1 & t<2) deltaf = steer_angle; end;
if (t>=2 & t<3) deltaf = -steer_angle; end;
if (t>=3 & t<4) deltaf = 0; end;
if (t>=4 & t<5) deltaf = -steer_angle; end;
if (t>=5 & t<6) deltaf = steer_angle; end;
if (t>=6) deltaf = 0; end;


% compute the slip angle
alphafl = deltaf - atan((L1*omegaz+vyfl)/vxfl);
alphafr = deltaf - atan((L1*omegaz+vyfr)/vxfr);
alpharl = atan((L2*omegaz-vyrl)/vxrl);
alpharr = atan((L2*omegaz-vyrr)/vxrr);
alphafl = deltaf - atan2(vyfl,vxfl);
alphafr = deltaf - atan2(vyfr,vxfr);
alpharl = deltar - atan2(vyrl,vxrl);
alpharr = deltar - atan2(vyrr,vxrr);

%Lateral forces acting on wheels
Fyfl = Cf*alphafl; 
Fyfr = Cf*alphafr;
Fyrl = Cr*alpharl;
Fyrr = Cr*alpharr;

%The lateral force is applied to the wheel
%unless it is locked while it still has significant velocity
% This does nothing if we do not lock out any of the wheels
if t>=break_on
    if vx>v_min
    Fyfl = Fyfl*(1-ifl);
    Fyfr = Fyfr*(1-ifr);
    Fyrl = Fyrl*(1-irl);
    Fyrr = Fyrr*(1-irr);
    end
end

% disturbance force
if t >= t_disturb && t <= t_disturb+0.15 
    Fd = Fyfd;
	climb = grade;
else
    Fd = 0;
    climb = grade;
end



FR = fr + (0.4e-7)*(60*vx/1000)^2;
B_r = 16;
taud = (Jrl + mw*R_w^2)/(B_r*R_w^2);
K1 = 1/(B_r*R_w^2);
Kp = 19/(K1*R_w);


%Cruise Control Parameters
e = r - vx;%(omega_rl+omega_rr)*R_w/2;
de = (-z+e)/taud;
T_controller = Kp*e + Ki*ie + Kd*de; %PID Control

if ie > 200
   ie = 0; 
end

Nr = cos(climb)*(L1- FR*h)*m*g/(L - mu*h); %Normal force on a rear tires
Nf = cos(climb)*(L2+ FR*h)*m*g/(L + mu*h); %Normal force on a front tires
Tmax = mu*Nr*R_w; %maximum torque tire can sustain

if T_controller >= Tmax
    Td = Tmax;
else
   Td = T_controller; 
end



% Fx = tractive force - rolling resistance force 
Fxfl = mu_slip(omega_fl,vxfl)*Nf- B_r*vx/4; % front-left traction force
Fxfr = mu_slip(omega_fr,vxfr)*Nf- B_r*vx/4; % front-right traction force
Fxrl = mu_slip(omega_rl,vxrl)*Nr - B_r*vx/4; % rear-left traction force
Fxrr = mu_slip(omega_rr,vxrr)*Nr - B_r*vx/4; % rear-right traction force

%Effective Torque
Tfl = (-mu_slip(omega_fl,vxfl)*Nf*R_w);
Tfr = (-mu_slip(omega_fr,vxfr)*Nf*R_w);
Trl = (Td - mu_slip(omega_rl,vxrl)*Nr*R_w);
Trr = (Td - mu_slip(omega_rr,vxrr)*Nr*R_w);


% braking forces at each wheel - note: use the vx at each wheel
if t>=break_on
    Fxbfl = ifl*mu*m*g*tanh(vxfl/0.01)/4; % front-left braking force
    Fxbfr = ifr*mu*m*g*tanh(vxfr/0.01)/4; % front-right braking force
    Fxbrl = irl*mu*m*g*tanh(vxrl/0.01)/4; % rear-left braking force
    Fxbrr = irr*mu*m*g*tanh(vxrr/0.01)/4; % rear-right braking force
else
    Fxbfl = 0; % front-left braking force
    Fxbfr = 0; % front-right braking force
    Fxbrl = 0; % rear-left braking force
    Fxbrr = 0; % rear-right braking force
end

%% System equations
% Vehicle Dynamics
vx_dot = (m*vy*omegaz + (Fxfl-Fxbfl+Fxfr-Fxbfr)*cos(deltaf) + (Fxrl-Fxbrl+Fxrr-Fxbrr) - (Fyfl+Fyfr)*sin(deltaf))/m;
vy_dot = (-m*vx*omegaz + Fyrl + Fyrr + (Fyfl+Fyfr)*cos(deltaf) + (Fxfl-Fxbfl+Fxfr-Fxbfr)*sin(deltaf) + Fd)/m;
omegaz_dot = (L1*(Fyfl+Fyfr)*cos(deltaf)+0.5*B*(Fyfr-Fyfl)*sin(deltaf)-L2*(Fyrl+Fyrr)+ ...
-L1*(Fxfl-Fxbfl+Fxfr-Fxbfr)*sin(deltaf)-0.5*B*(Fxfl-Fxbfl-Fxfr+Fxbfr)*cos(deltaf)+ ...
+0.5*B*(Fxrr-Fxbrr-Fxrl+Fxbrl)+Fd*L1)/Iz;
Xtdot = vx*cos(psit)-vy*sin(psit);
Ytdot = vx*sin(psit)+vy*cos(psit);
psidot = omegaz;

% Rotational Dynamics for Wheels
omega_fl_dot = (Tfl - R_w*Fxbfl)/Jrl;
omega_fr_dot = (Tfr -Fxbfr*(R_w))/Jrl;
omega_rl_dot = (Trl -Fxbrl*(R_w))/Jrl;
omega_rr_dot = (Trr -Fxbrr*(R_w))/Jrl;

% PID terms
iedot = e;
zdot = de;

xdot=[vx_dot;vy_dot;omegaz_dot;Xtdot;Ytdot;psidot; iedot; zdot; omega_fl_dot; omega_fr_dot; omega_rl_dot; omega_rr_dot];

% output equations
y(1) = alphafl;
y(2) = alphafr;
y(3) = alpharl;
y(4) = alpharr;
y(5) = Fyfl;
y(6) = Fyfr;
y(7) = Fyrl;
y(8) = Fyrr;
y(9) = Fd;
y(10) = Td;
y(11) = e;