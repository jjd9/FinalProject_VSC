function [xdot,y] = bicycle_model_ABS(t,x)
% Note: this is a 2D vehicle model that has braking and lateral forces at
% four wheels: fl, fr, rl, rr
% It is assumed that both front tires have same Cf, same for rear
% This is a simulation of fully-locked braking.
global L B R_w L1 L2 h Toggle_ABS slip_ref tau_lag Pbmax Jrl tau_b G
global m g mw Iz Cf Cr steer_angle alpha_maxf alpha_maxr drivetrain_damp
global Fyfd V_x Power_rated Gratio
global mu ifl ifr irl irr v_min fr break_on 

%Parse input
vx = x(1); vy = x(2); omegaz = x(3);
Xt = x(4); Yt = x(5); psit = x(6);
omega_fl = x(7); omega_fr = x(8); omega_rl = x(9); omega_rr = x(10); 
Pb_fl = x(11); Plag_fl = x(12);
Pb_fr = x(13); Plag_fr = x(14);
Pb_rl = x(15); Plag_rl = x(16);
Pb_rr = x(17); Plag_rr = x(18);


deltar = 0; % no rear steer
deltaf = 0; % no front steer

% compute the velocities at the four wheels
vxfl = vx - B*omegaz/2; vyfl = vy + L1*omegaz;
vxfr = vx + B*omegaz/2; vyfr = vy + L1*omegaz;
vxrl = vx - B*omegaz/2; vyrl = vy - L2*omegaz;
vxrr = vx + B*omegaz/2; vyrr = vy - L2*omegaz;


% determine slip state
s_fl = mu_slip(omega_fl,vxfl,1);
s_fr = mu_slip(omega_fr,vxfr,1);
s_rl = mu_slip(omega_rl,vxrl,1);
s_rr = mu_slip(omega_rr,vxrr,1);


if t>= break_on
    % hydraulic brake pressure model
    % lag in pressure buildup defined by taub (seconds)
    % Feedback for ABS
    % slip_ref is the specified slip reference value

    % Note, routine above gives + or - slip.
    slip_error_fl = slip_ref - abs(s_fl);
    slip_error_fr = slip_ref - abs(s_fr);  
    slip_error_rl = slip_ref - abs(s_rl);
    slip_error_rr = slip_ref - abs(s_rr);
    
    % Control: if slip is less than the reference value, increase pressure
    % if slip is greater than reference value, decrease pressure

    UPlag_fl = sign(slip_error_fl);
    UPlag_fr = sign(slip_error_fr);
    UPlag_rl = sign(slip_error_rl);
    UPlag_rr = sign(slip_error_rr);

    Klag = (6.895e5); % this is 100 psi expressed in Pa

    Plag_fl_dot = (-Plag_fl + Klag*UPlag_fl)/tau_lag; % lag
    Plag_fr_dot = (-Plag_fr + Klag*UPlag_fr)/tau_lag; % lag
    Plag_rl_dot = (-Plag_rl + Klag*UPlag_rl)/tau_lag; % lag
    Plag_rr_dot = (-Plag_rr + Klag*UPlag_rr)/tau_lag; % lag

    %Pressures can continue to grow until they exceed system capacity

    if (Pb_fl>Pbmax || Pb_fl<0),
        Pb_fl_dot = 0;
    else
        Pb_fl_dot = Plag_fl/tau_b;
    end

    if (Pb_fr>Pbmax || Pb_fr<0),
        Pb_fr_dot = 0;
    else
        Pb_fr_dot = Plag_fr/tau_b;
        
    end

    if (Pb_rl>Pbmax || Pb_rl<0),
        Pb_rl_dot = 0;
    else
        Pb_rl_dot = Plag_rl/tau_b;
    end

    if (Pb_rr>Pbmax || Pb_rr<0),
        Pb_rr_dot = 0;
    else
        Pb_rr_dot = Plag_rr/tau_b;
    end
else
    Plag_fl_dot = 0;
    Plag_fr_dot = 0;
    Plag_rl_dot = 0;
    Plag_rr_dot = 0;
    Pb_fl_dot = 0;
    Pb_fr_dot = 0;
    Pb_rl_dot = 0;
    Pb_rr_dot = 0;
    
end


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

ifl = 0; ifr = 0; irl = 0; irr = 0;

if t>=break_on
    if vx>v_min
    Fyfl = Fyfl*(1-ifl);
    Fyfr = Fyfr*(1-ifr);
    Fyrl = Fyrl*(1-irl);
    Fyrr = Fyrr*(1-irr);
    end
end

% disturbance force
if (t>=2 & t<=2.25)
    Fd = Fyfd;
else
    Fd = 0;
end

%Rolling Resistance
FR = fr + (0.4e-7)*(60*vx/1000)^2;

Nr = (L1- FR*h)*m*g/(L - mu*h); %Normal force on a rear tire
Nf = (L2+ FR*h)*m*g/(L + mu*h); 
Tmax = mu*Nr*R_w; %maximum torque tire can sustain

eff = 0.25; %drivetrain/motor efficiency
w_in = Gratio*(omega_rl+omega_rr)/2;
T_rated = Power_rated*eff/w_in;

if Tmax <= T_rated*Gratio
    Td = Tmax;
else
    Td = (T_rated)*Gratio;
end

% braking forces at each wheel - note: use the vx at each wheel
if t>=break_on
    % Assume different braking pressure goes to each wheel
    Tb_fl = G*abs(Pb_fl); 
    Tb_fr = G*abs(Pb_fr); 
    Tb_rl = G*abs(Pb_rl) ;
    Tb_rr = G*abs(Pb_rr); 
    Fxbfl = Tb_fl*sign(omega_fl)/R_w;
    Fxbfr = Tb_fr*sign(omega_fr)/R_w;
    Fxbrl = Tb_rl*sign(omega_rl)/R_w;
    Fxbrr = Tb_rr*sign(omega_rr)/R_w;

    Td=0; %Stop driving the wheels while braking
else
    Fxbfl = 0; % front-left braking force
    Fxbfr = 0; % front-right braking force
    Fxbrl = 0; % rear-left braking force
    Fxbrr = 0; % rear-right braking force
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


% system equations
vx_dot = (m*vy*omegaz + (Fxfl+Fxfr)*cos(deltaf) + (Fxrl+Fxrr) - (Fyfl+Fyfr)*sin(deltaf))/m;
vy_dot = (-m*vx*omegaz + Fyrl + Fyrr + (Fyfl+Fyfr)*cos(deltaf) + (Fxfl+Fxfr)*sin(deltaf) + Fd)/m;
omegaz_dot = (L1*(Fyfl+Fyfr)*cos(deltaf)+0.5*B*(Fyfr-Fyfl)*sin(deltaf)-L2*(Fyrl+Fyrr)+ ...
-L1*(Fxfl-Fxbfl+Fxfr-Fxbfr)*sin(deltaf)-0.5*B*(Fxfl-Fxfr)*cos(deltaf)+ ...
+0.5*B*(Fxrr-Fxrl)+Fd*L1)/Iz;

if ((omega_fl <= 0.0)&(R_w*Fxbfl>(Tfl)))
    omega_fl_dot =0;
else
    omega_fl_dot = (Tfl -R_w*Fxbfl)/Jrl;
end

if ((omega_fr <= 0.0)&(R_w*Fxbfr>(Tfr)))
    omega_fr_dot =0;
else
    omega_fr_dot = (Tfr -R_w*Fxbfr)/Jrl;
end

if ((omega_rl <= 0.0)&(R_w*Fxbrl>(Trl)))
    omega_rl_dot =0;
else
    omega_rl_dot = (Trl -R_w*Fxbrl)/Jrl;
end

if ((omega_rr <= 0.0)&(R_w*Fxbrr>(Trr)))
    omega_rr_dot =0;
else
    omega_rr_dot = (Trr -R_w*Fxbrr)/Jrl;
end

if vx <= 0 
   Xtdot = 0;
   Ytdot = 0;
else
Xtdot = vx*cos(psit)-vy*sin(psit);
Ytdot = vx*sin(psit)+vy*cos(psit);
end
psidot = omegaz;
xdot=[vx_dot;vy_dot;omegaz_dot;Xtdot;Ytdot;psidot; omega_fl_dot; omega_fr_dot; omega_rl_dot; omega_rr_dot; ...
    Pb_fl_dot; Plag_fl_dot; Pb_fr_dot; Plag_fr_dot; Pb_rl_dot; Plag_rl_dot; Pb_rr_dot; Plag_rr_dot];

%Output variable

y(1) = s_fl;
y(2)= s_fr;
y(3)= s_rl;
y(4)= s_rr;
y(5) = (Pb_rl+Pb_rr)/2;
y(6) = (Plag_rl+Plag_rr)/2;

