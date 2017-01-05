% forces_moments.m
%   Computes the forces and moments acting on the airframe. 
%
%   Output is
%       F     - forces
%       M     - moments
%       Va    - airspeed
%       alpha - angle of attack
%       beta  - sideslip angle
%       wind  - wind vector in the inertial frame
%
%  Revised:
%   2/2/2010 - RB 
%   5/14/2010 - RB
%   11/17/2014 - RWB (2D version)

function out = forces_moments(x, delta, wind, P)

    % relabel the inputs
    pn      = x(1);
    pd      = x(2);
    u       = x(3);
    w       = x(4);
    theta   = x(5);
    q       = x(6);
    delta_e = delta(1);
    delta_t = delta(2);
    w_ns    = wind(1); % steady wind - North
    w_ds    = wind(2); % steady wind - Down
    u_wg    = wind(3); % gust along body x-axis
    w_wg    = wind(4); % gust along body z-axis
    
    % convert steady inertial frame wind to the body frame
    % rotation from inertial frame to body frame
    R = [...
        cos(theta), -sin(theta);...
        sin(theta), cos(theta)];
    % compute wind vector in the body frame
    u_w = u_wg + R(1,:)*[w_ns; w_ds];
    w_w = w_wg + R(2,:)*[w_ns; w_ds];
    % compute wind vector in the inertial frame
    w_n = w_ns + R(:,1)'*[u_wg; w_wg];
    w_d = w_ds + R(:,2)'*[u_wg; w_wg];
    
    % compute the velocity relative to the air mass
    ur      = u-u_w;
    wr      = w-w_w;
    
    % compute airspeed Va, angle-of-attack alpha, side-slip beta
    Va    = sqrt(ur^2 + wr^2);
    alpha = atan2(wr,ur);
    qbar = 0.5*P.rho*Va^2;
    ca    = cos(alpha);
    sa    = sin(alpha);
   
    % compute gravitaional forces
    Force(1) = -P.mass*P.gravity*sin(theta);
    Force(2) =  P.mass*P.gravity*cos(theta);
    
    % compute Lift and Drag forces
    tmp1 = exp(-P.M*(alpha-P.alpha0));
    tmp2 = exp(P.M*(alpha+P.alpha0));
    sigma = (1+tmp1+tmp2)/((1+tmp1)*(1+tmp2));
    CL = (1-sigma)*(P.C_L_0+P.C_L_alpha*alpha);
    AR = 2;
    e = .9;
    CD = P.C_D_0 + 1/pi/e/AR*(P.C_L_0+P.C_L_alpha*alpha)^2;
    if alpha>=0, 
        CL = CL + sigma*2*sa*sa*ca;
    else
        CL = CL - sigma*2*sa*sa*ca;
    end
    
    % compute aerodynamic forces
    Force(1) = Force(1) + qbar*P.S_wing*(-CD*ca + CL*sa);
    Force(1) = Force(1) + qbar*P.S_wing*(-P.C_D_q*ca + P.C_L_q*sa)*P.c*q/(2*Va);
    
    Force(2) = Force(2) + qbar*P.S_wing*(-CD*sa - CL*ca);
    Force(2) = Force(2) + qbar*P.S_wing*(-P.C_D_q*sa - P.C_L_q*ca)*P.c*q/(2*Va);
     
    % compute aerodynamic torques
    
    Torque = qbar*P.S_wing*P.c*(P.C_m_0 + P.C_m_alpha*alpha);
    Torque = Torque + qbar*P.S_wing*P.c*P.C_m_q*P.c*q/(2*Va);


    % compute control forces
    Force(1) = Force(1) + qbar*P.S_wing*(-P.C_D_delta_e*ca+P.C_L_delta_e*sa)*delta_e;
    Force(2) = Force(2) + qbar*P.S_wing*(-P.C_D_delta_e*sa-P.C_L_delta_e*ca)*delta_e;
     
    % compute control torques
    Torque = Torque + qbar*P.S_wing*P.c*P.C_m_delta_e*delta_e;
    
    % compute propulsion forces
%    motor_temp = (P.k_motor*delta_t+Va)^2-Va^2; % revised model from book
    motor_temp = P.k_motor^2*delta_t^2-Va^2;
%   motor_temp = ( Va + delta_t*(P.k_motor - Va) )^2 - Va^2;
    Force(1) = Force(1) + 0.5*P.rho*P.S_prop*P.C_prop*motor_temp;
%  new propeller model
%    Force(1) = Force(1) + P.rho*P.S_prop*P.C_prop*(Va+delta_t*(P.k_motor-Va))*(delta_t*(P.k_motor-Va));
    
    
    out = [Force'; Torque; Va; alpha; w_n; w_d];
    
end



