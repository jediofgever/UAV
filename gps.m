
%   Compute the output of gps sensor


function y = gps(uu, P)

    % relabel the inputs
    NN = 0;
%    Va      = uu(1+NN);
%    alpha   = uu(2+NN);
%    wn      = uu(3+NN);
%    wd      = uu(4+NN);
    NN = NN + 4;
    pn      = uu(1+NN);
    pd      = uu(2+NN);
    u       = uu(3+NN);
    w       = uu(4+NN);
    theta   = uu(5+NN);
%    q       = uu(6+NN);
    NN = NN + 6;
    t       = uu(1+NN);
    
    % persistent variables that define random walk of GPS sensors
    persistent eta_n;
    persistent eta_h;
    
    
    if t==0,  % initialize persistent variables
        eta_n = 0;
        eta_h = 0;
    else      % propagate persistent variables
        eta_n = exp(-P.beta_gps*P.Ts_gps)*eta_n + P.sigma_n_gps*randn;
        eta_h = exp(-P.beta_gps*P.Ts_gps)*eta_h + P.sigma_h_gps*randn;
    end


    % construct North, East, and altitude GPS measurements
    y_gps_n = pn + eta_n;
    y_gps_h = -pd + eta_h; 
    
    % construct groundspeed and course measurements
    y_gps_Vg     = u*cos(theta)+w*sin(theta) + P.sigma_Vg_gps*randn;

    % construct total output
    y = [...
        y_gps_n;...
        y_gps_h;...
        y_gps_Vg;...
        ];
    
end



