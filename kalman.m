
function X_filtered = kalman(X, dt)
  % TODO
  % Write General Kalman Filter
  X_filtered = zeros(12, 1);
  X_filtered(1:6) = transKalman(X(1:3), dt);
  X_filtered(7:12) = rotKalman(X(7:12), dt);
end

function next= transKalman(currX, Ts)

    %-------------------------------
    %-------------------------------
    %---- Predict Stage ------------
    %-------------------------------
    %-------------------------------
    persistent Xhat
    if isempty(Xhat)
        Xhat = [0 0 0 0 0 0]';
    end
    persistent P
    if isempty(P)
        P=eye(6);
    end

    p = 0.1;

    %Process Noisepersistent P
    Q = diag([p^2 p^2 p^2 p^2 p^2 p^2]);

    %A matrix
    A =  [1   0  0  Ts  0   0
          0   1  0  0   Ts  0
          0   0  1  0   0   Ts
          0   0  0  1   0   0
          0   0  0  0   1   0
          0   0  0  0   0   1];

    %odutput these values with a unit delay (for algebraic loop), apply initial conditions
    Xhatm = A*Xhat;
    Pm = A*P*A' + Q;

    %-------------------------------
    %-------------------------------
    %---- Estimate Stage -----------
    %-------------------------------
    %-------------------------------
    %C matrix
     C = [1 0 0 0 0 0;
          0 1 0 0 0 0;
          0 0 1 0 0 0];

    %Measurement noise
    m=1;
    R = diag([m^2 m^2 m^2]);

    %Measurement Z
    Y = currX;

    %Compute Kalman Gain
    Kalman_Gain = Pm*C'*inv(C*Pm*C'+R); % 6x6 matrix

    %compute state space correction based on measurement
    Xhat = Xhatm + Kalman_Gain*(Y - C*Xhatm); % 6x1 vector of states

    %compute covariance correction
    P = (eye(6) - Kalman_Gain*C)*Pm; % 6x6 covariance matrix

   next = Xhat;
end

function next =  rotKalman(currX, Ts)

    %-------------------------------
    %-------------------------------
    %---- Predict Stage ------------
    %-------------------------------
    %-------------------------------
    persistent Xhat
    if isempty(Xhat)
        Xhat = [0 0 0 0 0 0]';
    end
    persistent P
    if isempty(P)
        P=eye(6);
    end

    p = 0.01;

    %Process Noise
    Q = diag([p^2 p^2 p^2 p^2 p^2 p^2]);

    %A matrix
    A =  [1   0  0  Ts  0   0
          0   1  0  0   Ts  0
          0   0  1  0   0   Ts
          0   0  0  1   0   0
          0   0  0  0   1   0
          0   0  0  0   0   1];

    %odutput these values with a unit delay (for algebraic loop), apply initial conditions
    Xhatm = A*Xhat;
    Pm = A*P*A' + Q;

    %-------------------------------
    %-------------------------------
    %---- Estimate Stage -----------
    %-------------------------------
    %-------------------------------
    %C matrix
     C = eye(6);

    %Measurement noise
    m1=2;
    m2=1;
    m1_yaw=10;
    m2_yaw=10;
    R = diag([m1^2 m1^2 m1_yaw^2 m2^2 m2^2 m2_yaw^2]);

    %Measurement Z
    Y = currX;

    %Compute Kalman Gain
    Kalman_Gain = Pm*C'*inv(C*Pm*C'+R); % 6x6 matrix

    %compute state space correction based on measurement
    Xhat = Xhatm + Kalman_Gain*(Y - C*Xhatm); % 6x1 vector of states

    %compute covariance correction
    P = (eye(6) - Kalman_Gain*C)*Pm; % 6x6 covariance matrix

    next = Xhat;
end
