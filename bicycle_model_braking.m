function [xdot, y] = bicycle_model_braking(t,x)
% Note: this is a 2D vehicle model that has braking and lateral forces at
% four wheels: fl, fr, rl, rr
% It is assumed that both front tires have same Cf, same for rear
% This is a simulation of fully-locked braking.
global L B R_w L1 L2 h Jrl turn_time drivetrain_damp 
global m g mw Iz Cf Cr steer_angle alpha_maxf alpha_maxr
global Fyfd V_x Power_rated Gratio
global mu ifl ifr irl irr v_min fr break_on t_disturb

%Parse input
vx = x(1); vy = x(2); omegaz = x(3);
Xt = x(4); Yt = x(5); psit = x(6);
omega_fl = x(7); omega_fr = x(8); omega_rl = x(9); omega_rr = x(10); 

if steer_angle ~= 0
   vx = V_x;
   vy = 0;
end

deltar = 0; % no rear steer

% compute the velocities at the four wheels
vxfl = vx - B*omegaz/2; vyfl = vy + L1*omegaz;
vxfr = vx + B*omegaz/2; vyfr = vy + L1*omegaz;
vxrl = vx - B*omegaz/2; vyrl = vy - L2*omegaz;
vxrr = vx + B*omegaz/2; vyrr = vy - L2*omegaz;


% double lane change (nothing will happen when steer_angle is set to 0)
if (t<turn_time + 1) 
    deltaf = 0; 
end;
if (t>=turn_time + 1 & t<turn_time + 2) deltaf = steer_angle; end;
if (t>=turn_time + 2 & t<turn_time + 3) deltaf = -steer_angle; end;
if (t>=turn_time + 3 & t<turn_time + 4) deltaf = 0; end;
if (t>=turn_time + 4 & t<turn_time + 5) deltaf = -steer_angle; end;
if (t>=turn_time + 5 & t<turn_time + 6) deltaf = steer_angle; end;
if (t>=turn_time + 6) 
    deltaf = 0; 
end;


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
if (t>=t_disturb & t<=t_disturb+0.25)
    Fd = Fyfd;
else
    Fd = 0;
end

%Rolling Resistance
FR = fr + (0.4e-7)*(60*vx/1000)^2;

Nr = (L1- FR*h)*m*g/(L - mu*h); %Normal force on a rear tire
Nf = (L2+ FR*h)*m*g/(L + mu*h); 
Tmax = mu*Nr*R_w; %maximum torque tire can sustain

eff = 0.85; %drivetrain/motor efficiency
w_in = Gratio*(omega_rl+omega_rr)/2;
T_rated = Power_rated*eff/w_in;

if Tmax <= T_rated*Gratio
    Td = Tmax;
else
    Td = (T_rated)*Gratio;
end

if t>= break_on && sum([ifl,ifr,irl,irr])~= 0
    Td = 0;
end



% Fx = tractive force - rolling resistance force
Fxfl = (mu_slip(omega_fl,vxfl)*Nf- FR*m*g*tanh(vx/0.1)/4); % front-left traction force
Fxfr = (mu_slip(omega_fr,vxfr)*Nf- FR*m*g*tanh(vx/0.1)/4); % front-right traction force
Fxrl = (mu_slip(omega_rl,vxrl)*Nr - FR*m*g*tanh(vx/0.1)/4); % rear-left traction force
Fxrr = (mu_slip(omega_rr,vxrr)*Nr - FR*m*g*tanh(vx/0.1)/4); % rear-right traction force

%Effective Torque
Tfl = (-mu_slip(omega_fl,vxfl)*Nf*R_w - drivetrain_damp *omega_fl);
Tfr = (-mu_slip(omega_fr,vxfr)*Nf*R_w- drivetrain_damp *omega_fr);
Trl = (Td - mu_slip(omega_rl,vxrl)*Nr*R_w- drivetrain_damp *omega_rl);
Trr = (Td - mu_slip(omega_rr,vxrr)*Nr*R_w- drivetrain_damp *omega_rr);

%Force and Torque Explanation
%Since I have four wheels and I am only driving the back two, my  driving/tractive/rolling force for the front wheels are Ftx - Froll, and my rear wheel forces are (DriveTorque / wheel_radius) + Ftx - Froll. And my torques for the rear wheels are DriveTorque - Ftx*wheelradius - (vertical distance to CM)*Froll, and my torques for the front wheels are +Ftx*wheelradius - (vertical distance to CM)*Froll . The reason I have Ftx*wheelradius as positive for the front wheels is that this is the torque that is effectively "driving" the front wheels to rotate, whereas for the rear wheels which I am actively driving, the Ftx*wheelradius will detract from this torque.


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

% system equations
vx_dot = (m*vy*omegaz + (Fxfl-Fxbfl+Fxfr-Fxbfr)*cos(deltaf) + (Fxrl-Fxbrl+Fxrr-Fxbrr) - (Fyfl+Fyfr)*sin(deltaf))/m;
vy_dot = (-m*vx*omegaz + Fyrl + Fyrr + (Fyfl+Fyfr)*cos(deltaf) + (Fxfl-Fxbfl+Fxfr-Fxbfr)*sin(deltaf) + Fd)/m;
omegaz_dot = (L1*(Fyfl+Fyfr)*cos(deltaf)+0.5*B*(Fyfr-Fyfl)*sin(deltaf)-L2*(Fyrl+Fyrr)+ ...
-L1*(Fxfl-Fxbfl+Fxfr-Fxbfr)*sin(deltaf)-0.5*B*(Fxfl-Fxbfl-Fxfr+Fxbfr)*cos(deltaf)+ ...
+0.5*B*(Fxrr-Fxbrr-Fxrl+Fxbrl)+Fd*L1)/Iz;


omega_fl_dot = (Tfl - R_w*Fxbfl)/Jrl;
omega_fr_dot = (Tfr -Fxbfr*(R_w))/Jrl;
omega_rl_dot = (Trl -Fxbrl*(R_w))/Jrl;
omega_rr_dot = (Trr -Fxbrr*(R_w))/Jrl;

Xtdot = vx*cos(psit)-vy*sin(psit);
Ytdot = vx*sin(psit)+vy*cos(psit);
psidot = omegaz;
xdot=[vx_dot;vy_dot;omegaz_dot;Xtdot;Ytdot;psidot; omega_fl_dot; omega_fr_dot; omega_rl_dot; omega_rr_dot];
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
