%Define Ground Vehicle Data for 2004 Envoy XL
global L B R_w L1 L2 h Jrl drivetrain_damp
global m mw g Iz Cf Cr steer_angle alpha_maxf alpha_maxr
global Fyfd Xtdon Xtdoff Power_rated Gratio
global mu ifl ifr irl irr v_min fr
global K1 tau t_cruise Kp Ki Kd taud
% Baseline values
g = 9.81;
% Geometric vehicle parameters (in meters)
L = 3.277;  % wheel base
L1 = 1.65; %approximating car's center of mass to be closer to back of car
L2 = L-L1;
h = 0.672; %Distance to cars center of mass from ground
B = 1.905;  % rear axle track width
R_w = 0.4318/2;  % wheel radius

% Inertia parameters
m = 2250;        % total mass, kg
mw = 10; %Mass of a wheel, kg
Jrl = 0.543;%0.5*mw*R_w^2;
W = m*g;			% weight, N
Wf = L2*W/L;		% static weight on front axle
Wr = L1*W/L;		% static weight on rear axle
iyaw = 0.992;		% yaw dynamic index
% the following is a defined relation between yaw dynamic index and 
% the yaw moment of inertia (see Dixon reference and table of car specs)
Iz = 1*iyaw*m*L1*L2;		% moment of inertia about z, kg-m^2


% Refer to Wong, Section 1.4 for guide to the following parameters
CCf = 0.171*180/pi;	% front corning stiffness coefficient, /rad
CCr = 0.181*180/pi;	% rear corning stiffness coefficient, /rad
Cf = CCf*Wf/2;		% corning stiffness per tire, N/rad (front)
Cr = 0.5*CCr*Wr/2;		% rear cornering stiffness per tire, N/rad (rear)

fr = 0.0136; % tire rolling resistance

% Peak braking mu
mu = 0.7; % coefficient of friction - peak value
% determine the max values of alpha on front and rear
alpha_maxf = mu*m*g/(Cf*4); % 
alpha_maxr = mu*m*g/(Cr*4); % 

% Criteria for stability (Rocard basic model - no steer angle)
Rfactor = Cr*L2 - Cf*L1;
% critical velocity - if Rfactor is positive, there is no critical speed
if Rfactor<0
    Vc = sqrt(2*Cf*Cr*L*L/(m*(Cf*L1-Cr*L2)));
else
    Vc = 0;
end

% the locked condition should only lead to loss of lateral
% force if the (absolute) wheel velocity is above a certain value
v_min = 1;

Gratio = 3.5;
Power_rated = 2.5e5; %Rated power for car, Watts

Id = .35;%0.135581795;%0.71; %Inertia from drivetrain, arbitrarily defined to get steady state acceleration response
Jrl = Jrl + Id*(Gratio)^2; %Effective inertia at wheels
drivetrain_damp = 0.6;

%Aerodynamics
rho=1.202; %density of air, [kg/m^3]  
A=2.150; %Frontal Area of car [m^2]
Cd=0.4; %Aerodynamic Drag