% -----------------------------------------------------
% tricycle_state.m 
% -----------------------------------------------------
function [xb, yb, xfrw, yfrw, xflw, yflw, xrrw, yrrw, xrlw, yrlw] = car_state(q)

global L B R_w L1 L2 deltaf

if deltaf ~= []
    delta = deltaf;
else
   delta = 0; 
end
x = q(4); y = q(5); psi = q(6);

% locates front-center point
xfc = x + L*cos(psi);
yfc = y + L*sin(psi);

% locates four corners
xfl = xfc - 0.5*B*sin(psi);
yfl = yfc + 0.5*B*cos(psi);
xfr = xfc + 0.5*B*sin(psi);
yfr = yfc - 0.5*B*cos(psi);
xrl = x - 0.5*B*sin(psi);
yrl = y + 0.5*B*cos(psi);
xrr = x + 0.5*B*sin(psi);
yrr = y - 0.5*B*cos(psi);

% end points of the front-steered wheels
xfwf_L = xfl + R_w*cos(psi+delta);
yfwf_L = yfl + R_w*sin(psi+delta);
xfwr_L = xfl - R_w*cos(psi+delta);
yfwr_L = yfl - R_w*sin(psi+delta);

xfwf_R = xfr + R_w*cos(psi+delta);
yfwf_R = yfr + R_w*sin(psi+delta);
xfwr_R = xfr - R_w*cos(psi+delta);
yfwr_R = yfr - R_w*sin(psi+delta);


% end points of the rear-left wheel
xrlwf = xrl + R_w*cos(psi);
yrlwf = yrl + R_w*sin(psi);
xrlwr = xrl - R_w*cos(psi);
yrlwr = yrl - R_w*sin(psi);

% end points of the rear-right wheel
xrrwf = xrr + R_w*cos(psi);
yrrwf = yrr + R_w*sin(psi);
xrrwr = xrr - R_w*cos(psi);
yrrwr = yrr - R_w*sin(psi);

% define the states

% front center point (not returned)
qfc = [xfc, yfc];

% body x-y points
xb = [xfl, xfr, xrr, xrl, xfl];
yb = [yfl, yfr, yrr, yrl, yfl];

% front wheel x-y points
xflw = [xfwf_L, xfwr_L];
yflw = [yfwf_L, yfwr_L];
xfrw = [xfwf_R, xfwr_R];
yfrw = [yfwf_R, yfwr_R];

% rear-left wheel x-y points
xrlw = [xrlwf, xrlwr];
yrlw = [yrlwf, yrlwr];

% rear-right wheel x-y points
xrrw = [xrrwf, xrrwr];
yrrw = [yrrwf, yrrwr];

    