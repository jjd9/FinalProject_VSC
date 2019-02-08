function mu_s = mu_slip(omega,vx, out)
    global R_w
    SLIP= (0:.05:1.0); % this creates a vector of slip values 0 to 1, stepping 0.05
    MU = 0.7*[0 .4 .8 .97 1.0 .98 .96 .94 .92 .9 .88 .855 .83 .81 .79 .77 .75 .73 .72 .71 .7]; 
    Nom = R_w*omega - vx;
    den = max(R_w*omega,vx);
    if den == 0
        mu_s = sign(Nom)*.7;
        slip = 1;
    else
        slip = Nom/den;
        mu_ex = 0.7; % for extrapolation, keep mu at nominal value
        
        mu_s = sign(slip)*interp1(SLIP,MU,abs(slip),'linear',mu_ex);
    if exist('out') 
        if abs(slip)>1
            slip = sign(slip)*1;
        end
       mu_s = slip; 
    end
    end
