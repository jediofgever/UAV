function y = autopilot(uu,P)
%
% autopilot for mavsim
% 
% Modification History:

%   
 
    % process inputs
    NN = 0;
    pn       = uu(1+NN);  % inertial North position
    h        = uu(2+NN);  % altitude
    Va       = uu(3+NN); 
    alpha    = uu(4+NN);
    theta    = uu(5+NN);  % pitch angle
    q        = uu(6+NN); % body frame pitch rate
    Vg       = uu(7+NN); % ground speed
    wn       = uu(8+NN); % wind North
    by       = uu(9+NN); % y-gyro bias
    NN = NN+9;
    Va_c     = uu(1+NN);  % commanded airspeed (m/s)
    h_c      = uu(2+NN);  % commanded altitude (m)
    NN = NN+2;
    t        = uu(1+NN);   % time
    
    [delta, x_command] = autopilot_no_state_machine(Va_c,h_c,Va,h,theta,q,t,P);
    y = [delta; x_command];
end
    
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% autopilot_no_state_machine.  Works well for overpowered aerosonde
%   - autopilot defined in the uavbook
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [delta, x_command] = autopilot_no_state_machine(Va_c,h_c,Va,h,theta,q,t,P)

    
    %----------------------------------------------------------
    % longitudinal autopilot
    if t==0,
        delta_t = airspeed_with_throttle_hold(Va_c, Va, 1, P);
        theta_c = altitude_hold(h_c, h, 1, P);
    else
        delta_t = airspeed_with_throttle_hold(Va_c, Va, 0, P);
        theta_c = altitude_hold(h_c, h, 0, P);
    end
    
    delta_e = pitch_hold(theta_c, theta, q, P);
    % artificially saturation delta_t
    delta_t = sat(delta_t,1,0);
 
    
    %----------------------------------------------------------
    % create outputs
    
    % control outputs
    delta = [delta_e; delta_t];
    % commanded (desired) states
    x_command = [...
        0;...                    % pn
        h_c;...                  % h
        Va_c;...                 % Va
        0;...                    % alpha
        theta_c;
        0;...                    % q
        ];
            
    y = [delta; x_command];
 
    end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autopilot functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% pitch_hold
%   - regulate pitch using elevator
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_e = pitch_hold(theta_c, theta, q, P)
 
  % compute the current error
  error = theta_c - theta;
  
  % proportional term
  up = P.pitch_kp * error;
  
  % derivative term
  ud = -P.pitch_kd * q;
  
  % implement PID control
  delta_e = sat(up + ud, 45*pi/180, -45*pi/180);
  
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% airspeed_with_pitch_hold
%   - regulate airspeed using pitch angle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function theta_c = airspeed_with_pitch_hold(Va_c, Va, flag, P)
  persistent integrator;
  persistent error_d1;
  % initialize persistent variables at beginning of simulation
  if flag==1,
      integrator = 0; 
      error_d1   = 0; 
  end
 
  % compute the current error
  error = Va_c - Va;
  
  % update the integrator
  integrator = integrator + (P.Ts/2)*(error + error_d1); % trapazoidal rule
  
  % proportional term
  up = P.airspeed_pitch_kp * error;
  
  % integral term
  ui = P.airspeed_pitch_ki * integrator;
  
  % implement PID control
  theta_c = sat(up + ui, P.theta_c_max, -P.theta_c_max);
  
  % implement integrator antiwindup
  if P.airspeed_pitch_ki~=0,
    theta_c_unsat = up + ui;
    k_antiwindup = P.Ts/P.airspeed_pitch_ki;
    integrator = integrator + k_antiwindup*(theta_c-theta_c_unsat);
  end

  % update persistent variables
  error_d1 = error;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% airspeed_with_throttle_hold
%   - regulate airspeed using throttle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_t = airspeed_with_throttle_hold(Va_c, Va, flag, P)
  persistent integrator;
  persistent error_d1;
  % initialize persistent variables at beginning of simulation
  if flag==1,
      integrator = 0; 
      error_d1   = 0; 
  end
 
  % compute the current error
  error = Va_c - Va;
  
  % update the integrator
  integrator = integrator + (P.Ts/2)*(error + error_d1); % trapazoidal rule
    
  % proportional term
  up = P.airspeed_throttle_kp * error;
  
  % integral term
  ui = P.airspeed_throttle_ki * integrator;
    
  % implement PID control
  delta_t = sat(up + ui, 1, 0);
  
  % implement integrator anti-windup
  if P.airspeed_throttle_ki~=0,
    delta_t_unsat = up + ui;
    k_antiwindup = P.Ts/P.airspeed_throttle_ki;
    integrator = integrator + k_antiwindup*(delta_t-delta_t_unsat);
  end
  
  % update persistent variables
  error_d1 = error;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% altitude_hold
%   - regulate altitude using pitch angle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function theta_c = altitude_hold(h_c, h, flag, P)
  persistent integrator;
  persistent error_d1;
  persistent hdot;
  persistent hdot_d1;
  persistent h_d1;

  % initialize persistent variables at beginning of simulation
  if flag==1,
      integrator = 0; 
      error_d1   = 0; 
      hdot = 0;
      hdot_d1 = 0;
      h_d1 = 0;
  end
 
  % compute the current error
  error = h_c - h;
  
  % update the integrator
    integrator = integrator + (P.Ts/2)*(error + error_d1); % trapazoidal rule
  
  % update the differentiator
  hdot = (2*P.tau-P.Ts)/(2*P.tau+P.Ts)*hdot_d1...
      + (2/(2*P.tau+P.Ts))*(h - h_d1);

  % proportional term
  up = P.altitude_kp * error;
  
  % integral term
  ui = P.altitude_ki * integrator;
  
  % derivative gain
  ud = P.altitude_kd * hdot;
  
  % implement PID control
    theta_c = sat(up + ui + ud, P.theta_c_max, -P.theta_c_max);
  
  % implement integrator anti-windup
  if P.altitude_ki~=0,
    theta_c_unsat = up + ui + ud;
    k_antiwindup = P.Ts/P.altitude_ki;
    integrator = integrator + k_antiwindup*(theta_c-theta_c_unsat);
  end
  
  % update persistent variables
  error_d1 = error;
  hdot_d1 = hdot;
  h_d1 = h;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sat
%   - saturation function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function out = sat(in, up_limit, low_limit)
  if in > up_limit,
      out = up_limit;
  elseif in < low_limit;
      out = low_limit;
  else
      out = in;
  end
end
  
 