% estimate_states
%   - estimate the MAV states using gyros, accels, pressure sensors, and
%   GPS.
%
% Outputs are:
%   pn_hat    - estimated North position, 
%   h_hat     - estimated altitude, 
%   Va_hat    - estimated airspeed, 
%   alpha_hat - estimated angle of attack
%   theta_hat - estimated pitch angel, 
%   q_hat     - estimated pitch rate, 
%   Vg_hat    - estimated ground speed, 
%   wn_hat    - estimate of North wind, 
%   by_hat    - estimate of y-axis gyro bias
% 
% 
% Modified:  3/15/2010 - RB
%            5/18/2010 - RB
%            11/18/2014 - RWB
%

function xhat = estimate_states(uu, P)

   % rename inputs
   gyro_y      = uu(1);
   accel_x     = uu(2);
   accel_z     = uu(3);
   static_pres = uu(4);
   diff_pres   = uu(5);
   gps_n       = uu(6);
   gps_h       = uu(7);
   gps_Vg      = uu(8);
   t           = uu(9);
 
   %[xhat, P_cov] = EKF_direct(gyro_y,accel_x,accel_z,static_pres,diff_pres,gps_n,gps_Vg,t,P);
   [xhat, P_cov] = EKF_indirect(gyro_y,accel_x,accel_z,static_pres,diff_pres,gps_n,gps_Vg,t,P);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% EKF_direct - direct Kalman Filter
function[xhat, Pcov] = EKF_direct(gyro_y, accel_x, accel_z, static_pres,diff_pres, gps_n, gps_Vg, t,P)
    persistent alpha  % constant for low pass filter - only compute once
    persistent lpf_gyro_y   % low pass filter of y-gyro
    persistent x_hat         % estimate of states
    persistent P_cov        % error covariance of stats
    persistent gps_n_old  % last measurement of gps_n - used to detect new GPS signal
    persistent gps_Vg_old % last measurement of gps_Vg - used to detect new GPS signal
    if t==0,
        lpf_a = 50;
        alpha = exp(-lpf_a*P.Ts);
        lpf_gyro_y = 0;
        x_hat = P.x_hat0;
        P_cov = P.P_cov0;
        gps_n_old  = -9999;
        gps_Vg_old = -9999;
    end
    
    % prediction step
    N = 10;
    for i=1:N,
        pn    = x_hat(1);
        h     = x_hat(2);
        u     = x_hat(3);
        w     = x_hat(4);
        theta = x_hat(5);
        by    = x_hat(6);
        wn    = x_hat(7);
        
        f = [...
            cos(theta)*u + sin(theta)*w;...% + wn;...
            sin(theta)*u - cos(theta)*w;...
            by*w - sin(theta)*P.gravity;...
            -by*u + cos(theta)*P.gravity;...
            -by;...
            0;...
            0;...
            ];
        G = [...
            0, 0, 0;...
            0, 0, 0;...
            -w, 1, 0;...
            u, 0, 1;...
            1, 0, 0;...
            0, 0, 0;...
            0, 0, 0;...
            ];
        A = [...
            0, 0, cos(theta), sin(theta), (-sin(theta)*u+cos(theta)*w), 0, 0;...
            0, 0, sin(theta), -cos(theta), (cos(theta)*u+sin(theta)*w), 0, 0;...
            0, 0, 0, -(gyro_y-by), -P.gravity*cos(theta), w, 0;...
            0, 0, (gyro_y-by), 0, -P.gravity*sin(theta), -u, 0;...
            0, 0, 0, 0, 0, -1, 0;...
            0, 0, 0, 0, 0, 0, 0;...
            0, 0, 0, 0, 0, 0, 0;...
            ];
        Qs = diag([(P.sigma_gyro)^2, (P.sigma_accel)^2, (P.sigma_accel)^2]);
        x_hat = x_hat + (P.Ts/N)*(f+G*[gyro_y; accel_x; accel_z]);
        P_cov = P_cov + (P.Ts/N)*(A*P_cov + P_cov*A' + G*Qs*G' + P.Q_direct);
    end

    % measurement update
    
    % update fast sensors: static pressure, diff pressure, pseudo-measurements
        
        % static pressure
        h_static = P.rho*P.gravity*x_hat(2);
        C_static = [0, P.rho*P.gravity, 0, 0, 0, 0, 0];
        R = P.sigma_static_pres^2;
        L = P_cov*C_static'/(R+C_static*P_cov*C_static');
        P_cov = (eye(7)-L*C_static)*P_cov;
        x_hat = x_hat + L*(static_pres - h_static);
        
        % differential pressure
        ur = x_hat(3)-x_hat(7)*cos(x_hat(5));
        wr = x_hat(4)-x_hat(7)*sin(x_hat(5));
        h_diff = 0.5*P.rho*(ur^2+wr^2);
        C_diff = P.rho*[0, 0, ur, wr, ur*x_hat(7)*sin(x_hat(5))-wr*x_hat(7)*cos(x_hat(5)),...
            0, -ur*cos(x_hat(5))-wr*sin(x_hat(5))];
        R = P.sigma_diff_pres^2;
        L = P_cov*C_diff'/(R+C_diff*P_cov*C_diff');
        P_cov = (eye(7)-L*C_diff)*P_cov;
        x_hat = x_hat + L*(diff_pres - h_diff);
        

    % update slow sensors: GPS
    if   (gps_n~=gps_n_old)|(gps_Vg~=gps_Vg_old),
        % gps North position
        h_gps_n = x_hat(1);
        C_gps_n = [1, 0, 0, 0, 0, 0, 0];
        R = P.sigma_n_gps^2;
        L = P_cov*C_gps_n'/(R+C_gps_n*P_cov*C_gps_n');
        P_cov = (eye(7)-L*C_gps_n)*P_cov;
        x_hat = x_hat + L*(gps_n - h_gps_n);
        
        % gps ground speed
        h_Vg = x_hat(3)*cos(x_hat(5))+x_hat(4)*sin(x_hat(5));
        C_Vg = [0, 0, cos(x_hat(5)), sin(x_hat(5)), (-x_hat(3)*sin(x_hat(5))+x_hat(4)*cos(x_hat(5))), 0, 0];
        R = P.sigma_Vg_gps^2;
        L = P_cov*C_Vg'/(R+C_Vg*P_cov*C_Vg');
        P_cov = (eye(7)-L*C_Vg)*P_cov;
        x_hat = x_hat + L*(gps_Vg - h_Vg);  

        % update stored GPS signals
        gps_n_old      = gps_n;
        gps_Vg_old     = gps_Vg;
    end
    
    % construct output
    ur = x_hat(3)-x_hat(7)*cos(x_hat(5));
    wr = x_hat(4)-x_hat(7)*sin(x_hat(5));
    Vahat = sqrt(ur^2+wr^2);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               
    alphahat = atan(wr/ur);
    
    % low pass filter the gyro to get q
    lpf_gyro_y = alpha*lpf_gyro_y + (1-alpha)*gyro_y;
    
    xhat = [...
        x_hat(1);...          % pn
        x_hat(2);...          % h
        Vahat;...
        alphahat;...
        x_hat(5);...          % theta
        lpf_gyro_y-x_hat(6);...  % q
        x_hat(3)*cos(x_hat(5))+x_hat(4)*sin(x_hat(5));...  % Vg
        x_hat(6);...          % bn
        x_hat(7);...          % wn
        ];
    Pcov=P_cov;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% EKF_indirect - indirect Kalman Filter
function[xhat, Pcov] = EKF_indirect(gyro_y, accel_x, accel_z, static_pres,diff_pres, gps_n, gps_Vg, t,P)
    persistent alpha  % constant for low pass filter - only compute once
    persistent lpf_gyro_y   % low pass filter of y-gyro
    persistent x_hat         % estimate of states
    persistent x_tilde      % the error state
    persistent P_cov        % error covariance of stats
    persistent gps_n_old  % last measurement of gps_n - used to detect new GPS signal
    persistent gps_Vg_old % last measurement of gps_Vg - used to detect new GPS signal
    if t==0,
        lpf_a = 50;
        alpha = exp(-lpf_a*P.Ts);
        lpf_gyro_y = 0;
        x_hat = P.x_hat0;
        x_tilde = zeros(7,1);
        P_cov = P.P_cov0;
        gps_n_old  = -9999;
        gps_Vg_old = -9999;
    end
    
    % prediction step
    N = 10;
    for i=1:N,
        pn    = x_hat(1);
        h     = x_hat(2);
        u     = x_hat(3);
        w     = x_hat(4);
        theta = x_hat(5);
        by    = x_hat(6);
        wn    = x_hat(7);
        
        f = [...
            cos(theta)*u + sin(theta)*w;...% + wn;...
            sin(theta)*u - cos(theta)*w;...
            by*w - sin(theta)*P.gravity;...
            -by*u + cos(theta)*P.gravity;...
            -by;...
            0;...
            0;...
            ];
        G = [...
            0, 0, 0;...
            0, 0, 0;...
            -w, 1, 0;...
            u, 0, 1;...
            1, 0, 0;...
            0, 0, 0;...
            0, 0, 0;...
            ];
        A = [...
            0, 0, cos(theta), sin(theta), (-sin(theta)*u+cos(theta)*w), 0, 0;...
            0, 0, sin(theta), -cos(theta), (cos(theta)*u+sin(theta)*w), 0, 0;...
            0, 0, 0, -(gyro_y-by), -P.gravity*cos(theta), w, 0;...
            0, 0, (gyro_y-by), 0, -P.gravity*sin(theta), -u, 0;...
            0, 0, 0, 0, 0, -1, 0;...
            0, 0, 0, 0, 0, 0, 0;...
            0, 0, 0, 0, 0, 0, 0;...
            ];
        Qs = diag([(P.sigma_gyro)^2, (P.sigma_accel)^2, (P.sigma_accel)^2]);
        x_hat = x_hat + (P.Ts/N)*(f+G*[gyro_y; accel_x; accel_z]);
        x_tilde = x_tilde + (P.Ts/N)*A*x_tilde;
        P_cov = P_cov + (P.Ts/N)*(A*P_cov + P_cov*A' + G*Qs*G' + P.Q_indirect);
    end

    % measurement update
    
    % update fast sensors: static pressure, diff pressure, pseudo-measurements
        
        % static pressure
        h_static = P.rho*P.gravity*x_hat(2);
        C_static = [0, P.rho*P.gravity, 0, 0, 0, 0, 0];
        R = P.sigma_static_pres^2;
        L = P_cov*C_static'/(R+C_static*P_cov*C_static');
        P_cov = (eye(7)-L*C_static)*P_cov;
        x_tilde = x_tilde + L*(static_pres - h_static - C_static*x_tilde);
        % differential pressure
        ur = x_hat(3)-x_hat(7)*cos(x_hat(5));
        wr = x_hat(4)-x_hat(7)*sin(x_hat(5));
        h_diff = 0.5*P.rho*(ur^2+wr^2);
        C_diff = P.rho*[0, 0, ur, wr, ur*x_hat(7)*sin(x_hat(5))-wr*x_hat(7)*cos(x_hat(5)),...
            0, -ur*cos(x_hat(5))-wr*sin(x_hat(5))];
        R = P.sigma_diff_pres^2;
        L = P_cov*C_diff'/(R+C_diff*P_cov*C_diff');
        P_cov = (eye(7)-L*C_diff)*P_cov;
        x_tilde = x_tilde + L*(diff_pres - h_diff-C_diff*x_tilde);

    % update slow sensors: GPS
    if   (gps_n~=gps_n_old)|(gps_Vg~=gps_Vg_old),
        % gps North position
        h_gps_n = x_hat(1);
        C_gps_n = [1, 0, 0, 0, 0, 0, 0];
        R = P.sigma_n_gps^2;
        L = P_cov*C_gps_n'/(R+C_gps_n*P_cov*C_gps_n');
        P_cov = (eye(7)-L*C_gps_n)*P_cov;
        x_tilde = x_tilde + L*(gps_n - h_gps_n-C_gps_n*x_tilde);
        
        % gps ground speed
        h_Vg = x_hat(3);
        C_Vg = [0, 0, 1, 0, 0, 0, 0];
        R = P.sigma_Vg_gps^2;
        L = P_cov*C_Vg'/(R+C_Vg*P_cov*C_Vg');
        P_cov = (eye(7)-L*C_Vg)*P_cov;
        x_tilde = x_tilde + L*(gps_Vg - h_Vg-C_Vg*x_tilde);  

        % update stored GPS signals
        gps_n_old      = gps_n;
        gps_Vg_old     = gps_Vg;
    end
    % correct the estimate with the errors state
    x_hat = x_hat + x_tilde;
    x_tilde = zeros(7,1);
    
    % construct output
    ur = x_hat(3)-x_hat(7)*cos(x_hat(5));
    wr = x_hat(4)-x_hat(7)*sin(x_hat(5));
    Vahat = sqrt(ur^2+wr^2);
    alphahat = atan(wr/ur);
    
    % low pass filter the gyro to get q
    lpf_gyro_y = alpha*lpf_gyro_y + (1-alpha)*gyro_y;
    
    xhat = [...
        x_hat(1);...          % pn
        x_hat(2);...          % h
        Vahat;...
        alphahat;...
        x_hat(5);...          % theta
        lpf_gyro_y-x_hat(6);...   % q
        x_hat(3)*cos(x_hat(5))+x_hat(4)*sin(x_hat(5));...          % Vg = u
        x_hat(6);...          % bn
        x_hat(7);...          % wn
        ];
    Pcov=P_cov;
end
