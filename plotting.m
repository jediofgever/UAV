function plotMAVStateVariables(uu,P)
%
% modified 12/11/2009 - RB
% modified 11/18/2014 - RWB

    % process inputs to function
    NN = 0;
    pn          = uu(1+NN);             % North position (meters)
    h           = -uu(2+NN);            % altitude (meters)
    u           = uu(3+NN);             % body velocity along x-axis (meters/s)
    w           = uu(4+NN);             % body velocity along z-axis (meters/s)
    theta       = 180/pi*uu(5+NN);      % pitch angle (degrees)
    q           = 180/pi*uu(6+NN);     % body angular rate along y-axis (degrees/s)
    NN = NN+6;
    Va          = uu(1+NN);            % airspeed (m/s)
    alpha       = 180/pi*uu(2+NN);     % angle of attack (degrees)
    wn          = uu(3+NN);            % wind in the North direction
    wd          = uu(4+NN);            % wind in the Down direction
    NN = NN+4;
    pn_c        = uu(1+NN);            % commanded North position (meters)
    h_c         = uu(2+NN);            % commanded altitude (meters)
    Va_c        = uu(3+NN);            % commanded airspeed (meters/s)
    alpha_c     = 180/pi*uu(4+NN);     % commanded angle of attack (degrees)
    theta_c     = 180/pi*uu(5+NN);     % commanded pitch angle (degrees)
    q_c         = 180/pi*uu(6+NN);     % commanded body angular rate along y-axis (degrees/s)
    NN = NN+6;
    pn_hat      = uu(1+NN);            % estimated North position (meters)
    h_hat       = uu(2+NN);            % estimated altitude (meters)
    Va_hat      = uu(3+NN);            % estimated airspeed (meters/s)
    alpha_hat   = 180/pi*uu(4+NN);     % estimated angle of attack (degrees)
    theta_hat   = 180/pi*uu(5+NN);     % estimated pitch angle (degrees)
    q_hat       = 180/pi*uu(6+NN);     % estimated body angular rate along y-axis (degrees/s)
    Vg_hat      = uu(7+NN);            % estimated groundspeed
    by_hat      = uu(8+NN);            % estimated North wind
    wn_hat      = uu(9+NN);            % estimated y-gyro bias
    NN = NN+9;
    delta_e     = 180/pi*uu(1+NN);     % elevator angle (degrees)
    delta_t     = uu(2+NN);            % throttle setting (unitless)
    NN = NN+2;
    t           = uu(1+NN);            % simulation time
    
    by = P.bias_gyro_y;
    
    % define persistent variables 
    persistent pn_handle
    persistent h_handle
    persistent Va_handle
    persistent alpha_handle
    persistent theta_handle
    persistent q_handle
    persistent Vg_handle
    persistent wn_handle
    persistent by_handle
    persistent delta_e_handle
    persistent delta_t_handle
    

  % first time function is called, initialize plot and persistent vars
    if t==0,
        figure(2), clf

        subplot(6,2,1)
        hold on
        pn_handle = graph_y_yhat_yd(t, pn, pn_hat, pn_c, 'p_n', []);
        
        subplot(6,2,2)
        hold on
        Va_handle = graph_y_yhat_yd(t, Va, Va_hat, Va_c, 'V_a', []);

        subplot(6,2,3)
        hold on
        h_handle = graph_y_yhat_yd(t, h, h_hat, h_c, 'h', []);
  
        subplot(6,2,4)
        hold on
        alpha_handle = graph_y_yhat_yd(t, alpha, alpha_hat, alpha_c, '\alpha', []);

        subplot(6,2,5)
        hold on
        Vg_handle = graph_y_yhat(t, u, Vg_hat, 'V_g', []);

        subplot(6,2,6)
        hold on
        by_handle = graph_y_yhat(t, by, by_hat, 'b_y', []);

        subplot(6,2,8)
        hold on
        wn_handle = graph_y_yhat(t, wn, wn_hat, 'w_n', []);

        subplot(6,2,9)
        hold on
        theta_handle = graph_y_yhat_yd(t, theta, theta_hat, theta_c, '\theta', []);
        
        subplot(6,2,10)
        hold on
        delta_t_handle = graph_y(t, delta_t, [], 'b');
        ylabel('\delta_t')
    
        subplot(6,2,11)
        hold on
        q_handle = graph_y_yhat_yd(t, q, q_hat, q_c, 'q', []);
        
        subplot(6,2,12)
        hold on
        delta_e_handle = graph_y(t, delta_e, [], 'b');
        ylabel('\delta_e')
                
         
    % at every other time step, redraw state variables
    else 
       graph_y_yhat_yd(t, pn, pn_hat, pn_c, 'p_n', pn_handle);
       graph_y_yhat_yd(t, Va, Va_hat, Va_c, 'V_a', Va_handle);
       graph_y_yhat_yd(t, h, h_hat, h_c, 'h', h_handle);
       graph_y_yhat_yd(t, alpha, alpha_hat, alpha_c, '\alpha', alpha_handle);
       graph_y_yhat(t, u, Vg_hat, 'V_g', Vg_handle);
       graph_y_yhat(t, by, by_hat, 'b_y', by_handle);
       graph_y_yhat(t, wn, wn_hat, 'w_n', wn_handle);
       graph_y_yhat_yd(t, theta, theta_hat, theta_c, '\theta', theta_handle);
       graph_y(t, delta_t, delta_t_handle);
       graph_y_yhat_yd(t, q, q_hat, q_c, 'q', q_handle);
       graph_y(t, delta_e, delta_e_handle);
    end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% graph y with lable mylabel
function handle = graph_y(t, y, handle, color)
  
  if isempty(handle),
    handle    = plot(t,y,color);
  else
    set(handle,'Xdata',[get(handle,'Xdata'),t]);
    set(handle,'Ydata',[get(handle,'Ydata'),y]);
    %drawnow
  end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% graph y and yd with lable mylabel
function handle = graph_y_yd(t, y, yd, lab, handle)
  
  if isempty(handle),
    handle(1)    = plot(t,y,'b');
    handle(2)    = plot(t,yd,'g--');
    ylabel(lab)
    set(get(gca, 'YLabel'),'Rotation',0.0);
  else
    set(handle(1),'Xdata',[get(handle(1),'Xdata'),t]);
    set(handle(1),'Ydata',[get(handle(1),'Ydata'),y]);
    set(handle(2),'Xdata',[get(handle(2),'Xdata'),t]);
    set(handle(2),'Ydata',[get(handle(2),'Ydata'),yd]);
    %drawnow
  end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot the variable y in blue, its estimated value yhat in green, and its 
% desired value yd in red, lab is the label on the graph
function handle = graph_y_yhat(t, y, yhat, lab, handle)
  
  if isempty(handle),
    handle(1)   = plot(t,y,'b');
    handle(2)   = plot(t,yhat,'g');
    ylabel(lab)
    set(get(gca,'YLabel'),'Rotation',0.0);
  else
    set(handle(1),'Xdata',[get(handle(1),'Xdata'),t]);
    set(handle(1),'Ydata',[get(handle(1),'Ydata'),y]);
    set(handle(2),'Xdata',[get(handle(2),'Xdata'),t]);
    set(handle(2),'Ydata',[get(handle(2),'Ydata'),yhat]);
    %drawnow
  end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot the variable y in blue, its estimated value yhat in green, and its 
% desired value yd in red, lab is the label on the graph
function handle = graph_y_yhat_yd(t, y, yhat, yd, lab, handle)
  
  if isempty(handle),
    handle(1)   = plot(t,y,'b');
    handle(2)   = plot(t,yhat,'g--');
    handle(3)   = plot(t,yd,'r-.');
    ylabel(lab)
    set(get(gca,'YLabel'),'Rotation',0.0);
  else
    set(handle(1),'Xdata',[get(handle(1),'Xdata'),t]);
    set(handle(1),'Ydata',[get(handle(1),'Ydata'),y]);
    set(handle(2),'Xdata',[get(handle(2),'Xdata'),t]);
    set(handle(2),'Ydata',[get(handle(2),'Ydata'),yhat]);
    set(handle(3),'Xdata',[get(handle(3),'Xdata'),t]);
    set(handle(3),'Ydata',[get(handle(3),'Ydata'),yd]);     
    %drawnow
  end

%
%=============================================================================
% sat
% saturates the input between high and low
%=============================================================================
%
function out=sat(in, low, high)

  if in < low,
      out = low;
  elseif in > high,
      out = high;
  else
      out = in;
  end

% end sat  


