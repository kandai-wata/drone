classdef KalmanFilter < handle
  properties
    A;
    % B;
    C;
    Q;
    R;
    dt;
    Xhat;
    P;
  end

  methods
    function obj = KalmanFilter(init, dt)
      obj.A =[1 0 0 dt 0 0 0 0 0 0 0 0;
              0 1 0 0 dt 0 0 0 0 0 0 0;
              0 0 1 0 0 dt 0 0 0 0 0 0;
              0 0 0 1 0 0 dt 0 0 0 0 0;
              0 0 0 0 1 0 0 dt 0 0 0 0;
              0 0 0 0 0 1 0 0 dt 0 0 0;
              0 0 0 0 0 0 1 0 0 dt 0 0;
              0 0 0 0 0 0 0 1 0 0 dt 0;
              0 0 0 0 0 0 0 0 1 0 0 dt;
              0 0 0 0 0 0 0 0 0 1 0  0;
              0 0 0 0 0 0 0 0 0 0 0  0;
              0 0 0 0 0 0 0 0 0 0 0  0]; % 12*12
      % obj.B;
      obj.C = [ 1 0 0 0 0 0 0 0 0 0 0 0;
                0 1 0 0 0 0 0 0 0 0 0 0;
                0 0 1 0 0 0 0 0 0 0 0 0;
                0 0 0 0 0 0 1 0 0 0 0 0;
                0 0 0 0 0 0 0 1 0 0 0 0;
                0 0 0 0 0 0 0 0 1 0 0 0;
                0 0 0 0 0 0 0 0 0 1 0 0;
                0 0 0 0 0 0 0 0 0 0 1 0;
                0 0 0 0 0 0 0 0 0 0 0 1]; % 9*12
      transQ = 0.1^2;
      rotQ  = 0.01^2;
      obj.Q = diag([transQ transQ transQ transQ transQ transQ ...
                    rotQ rotQ rotQ rotQ rotQ rotQ]); % 12*12
      obj.R = diag([2^2 2^2 2^2 0.2^2 0.2^2 1^2 0.1^2 0.1^2 1^2]); % 6*6
      obj.P = eye(length(obj.A));
      obj.dt = dt;
      obj.Xhat = init;
    end

    % function next = update(obj, Y, U)
    function next = update(obj, fun, A, Y, U)
      obj.A = A;
      % Priori Estimate
      % Xhatm = obj.A*obj.Xhat; % 12*1
      dX1 = fun(obj.Xhat, U)*obj.dt;
      dX2 = fun(obj.Xhat+dX1/2, U)*obj.dt; 
      dX3 = fun(obj.Xhat+dX2/2, U)*obj.dt; 
      dX4 = fun(obj.Xhat+dX3, U)*obj.dt; 
      Xhatm = obj.Xhat+(dX1+2*dX2+2*dX3+dX4)/6;
      % Error Covariance
      Pm = obj.A*obj.P*(obj.A)' + obj.Q; % 12*12
      
      % Kalman Kalman
      G = Pm*(obj.C)'*inv(obj.C*Pm*(obj.C)'+obj.R); % 12*9
      
      %compute state space correction based on measurement
      obj.Xhat = Xhatm + G*(Y - obj.C*Xhatm); % 12*1
      %compute covariance correction
      obj.P = (eye(length(obj.A)) - G*obj.C)*Pm; % 12*12

      next = obj.Xhat;

    end
  end
end
