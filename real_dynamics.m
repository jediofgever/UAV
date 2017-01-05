% this is for construct estimated outputs using real states
    function xhat = true_states(uu,P)
    % we are going to estimate pn , h, Va, alfa, theta, q, Vg,  b_y, wn 
    % FROM AVALÝABLE REAL DATA
    % where;
    % pn: north position
    % h :altitude
    % Va :speed of air
    % alfa:angle of attack
    % theta:pitch angle
    % Vg: ground speed
    % b_y: gyro bias
    % wn: wind in north direction

    NN =0;
    pn = uu(1+NN);
    h = uu(2+NN);
    Va = uu(3+NN);
    alpha = uu(4+NN);
    theta = uu(5+NN);
    q = uu(6+NN);
   
    wd = uu(8+NN);
    wn = uu(9+NN);
    NN = NN+10;
    t = uu(1+NN);



                pnhat=pn;
                hhat=h;
                Vahat=Va;
                alphahat=alpha;
                thetahat=theta;
                qhat=q;
                gamma_a = theta-alpha;
                Vghat = sqrt((Va*cos(gamma_a)+wn)^2 + (-Va*sin(gamma_a)+wd)^2);
                byhat=P.bias_gyro_y;
                wnhat=wn;

       xhat = [ pnhat;...
                hhat;...
                Vahat;...
                alphahat;...
                thetahat;...
                qhat;...
                Vghat;...
                wnhat;...
                byhat;...
                ];
end

