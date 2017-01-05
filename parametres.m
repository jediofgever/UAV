P.gravity = 9.8;
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Params for Aersonade UAV
%physical parameters of airframe
P.mass = 13.5;
P.Jy   = 1.135;
% aerodynamic coefficients
P.S_wing        = 0.55;
P.b             = 2.8956;
P.c             = 0.18994;
P.S_prop        = 0.2027;
P.rho           = 1.2682;
P.k_motor       = 80;
P.k_T_P         = 0;
P.k_Omega       = 0;
P.e             = 0.9;

P.C_L_0         = 0.28;
P.C_L_alpha     = 3.45;
P.C_L_q         = 0.0;
P.C_L_delta_e   = -0.36;
P.C_D_0         = 0.03;
P.C_D_alpha     = 0.30;
P.C_D_p         = 0.0437;
P.C_D_q         = 0.0;
P.C_D_delta_e   = 0.0;
P.C_m_0         = -0.02338;
P.C_m_alpha     = -0.38;
P.C_m_q         = -3.6;
P.C_m_delta_e   = -0.5;
P.C_prop        = 1.0;
P.M             = 50;
P.epsilon       = 0.1592;
P.alpha0        = 0.4712;

% wind parameters
P.wind_n = 5;
P.wind_d = 0;
P.L_u = 200;
P.L_w = 50;
P.sigma_u = 1.06; 
P.sigma_w = .7;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% initial conditions
P.Va0    = 35;    % initial airspeed
P.pn0    = 0;  % initial North position
P.pd0    = -100;  % initial Down position (negative altitude)
P.u0     = P.Va0;  % initial velocity along body x-axis
P.w0     = 0;  % initial velocity along body z-axis
P.theta0 = 0;  % initial pitch angle
P.q0     = 0;  % initial body frame pitch rate

% autopilot sample rate
P.Ts = 0.01;

% autopilot gains
%----------------------------------------------------
% low level autopilot gains
P.tau = 5;  % gain on dirty derivative
P.theta_c_max = 25*pi/180; % maximum pitch angle command

% select gains for the pitch loop
   P.pitch_kp = -4;
   P.pitch_kd = -1;
   P.pitch_ki = 0.0;

% select gains for altitude loop
   P.altitude_kp = .05;
   P.altitude_ki = 0;%.005;
   P.altitude_kd = 0.005;
 
% airspeed hold using throttle
   P.airspeed_throttle_kp = 1.5;
   P.airspeed_throttle_ki = .2;

%---------------------------------------
% Chapter 7 - sensor parameters
    P.sigma_gyro = 0.13*pi/180; % standard deviation of gyros in rad/sec
    P.bias_gyro_y = 10*pi/180; % bias on y-gyro
    P.sigma_accel = 0.0025*9.8; % standard deviation of accelerometers in m/s^2
    P.sigma_static_pres = 0.01*1000; % standard deviation of static pressure sensor in Pascals
    P.sigma_diff_pres = 0.002*1000;  % standard deviation of diff pressure sensor in Pascals

% GPS parameters
    P.Ts_gps = 1; % sample rate of GPS in s
    P.beta_gps = 1/1100; % 1/s
    P.sigma_n_gps = 0.21;
    P.sigma_h_gps = 0.40;
    P.sigma_Vg_gps = 0.05;

%---------------------------------------
% Chapter 8 - EKF
    % initial conditions for direct EKF
    P.x_hat0  = [...
                P.pn0;...  % initial guess of north position
                -P.pd0;... % initial guess of altitude
                P.Va0;...  % initial guess of velocity along x-axis
                0;...      % initial guess of velocity along z-axis
                0;...      % initial guess of pitch angle
                0;...      % initial guess of gyro bias
                0;...      % initial guess of north wind velocity
                ];
     P.P_cov0 = diag([...
                     ( 5 )^2,... % initial variance of north estimate (m)
                     ( 5 )^2,... % initial variance of altitude estimate (m)
                     ( 20 )^2,... % initial variance of x-axis velocity (m/s)
                     ( 10 )^2,... % initial variance of z-axis velocity (m/s)
                     ( 20*pi/180 )^2,... % initial variance of pitch angle (rad)
                     ( 20*pi/180 )^2,... % initial variance of gyro bias (rad/s)
                     ( 10 )^2,... % initial variance of north wind (m/s)
                     ]);
    P.Q_direct = diag([...
            ( 0.1 )^2,... % variance of process noise in pn
            ( 0.1 )^2,... % variance of process noise in pd
            ( 0.001 )^2,... % variance of process noise in u
            ( 0.001 )^2,... % variance of process noise in w
            ( 0.001 )^2,... % variance of process noise in theta
            ( 0.01 )^2,... % variance of process noise in b_y
            ( 0.01 )^2,... % variance of process noise in wn
            ]);
     P.Q_indirect = diag([...
            ( 0.1 )^2,... % variance of process noise in pn
            ( 0.1 )^2,... % variance of process noise in pd
            ( 0.0001 )^2,... % variance of process noise in u
            ( 0.0001 )^2,... % variance of process noise in w
            ( 0.0001 )^2,... % variance of process noise in theta
            ( 0.01 )^2,... % variance of process noise in b_y
            ( 0.01 )^2,... % variance of process noise in wn
            ]);
           


