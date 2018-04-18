function f = sim_drone(pop)
%%  Parameter
m = 0.475;
armlen=0.125;
Ix = 2.7*10^(-3);
Iy = 2.9*10^(-3);
Iz = 5.3*10^(-3);
g=9.81;
% Simulation Parameter
dt = 1/400;
t_start=0;
t_end=15;
Time = (t_start:dt:t_end)';

% Initial Condition
X = zeros(12, 1);
Xref = [1 1 1]'';
U = [0 0 0 0]';
PWM = zeros(4,1);
U_hover=[m*g 0 0 0]';

% Place to be saved
X_store = zeros(length(Time), length(X));
U_store = zeros(length(Time), length(U));
PWM_store = zeros(length(Time), length(PWM));

% Filter
freq=50;
lpf = LowPassFilter(dt, freq);
kf = KalmanFilter(dt);

% Drone
drone = Drone(m, Ix, Iy, Iz, armlen, g, dt);
  A = [0   0   0   1   0   0   0   0   0   0   0   0;
       0   0   0   0   1   0   0   0   0   0   0   0;
       0   0   0   0   0   1   0   0   0   0   0   0;
       0   0   0   0   0   0   0   g   0   0   0   0;
       0   0   0   0   0   0   -g  0   0   0   0   0;
       0   0   0   0   0   0   0   0   0   0   0   0;
       0   0   0   0   0   0   0   0   0   1   0   0;
       0   0   0   0   0   0   0   0   0   0   1   0;
       0   0   0   0   0   0   0   0   0   0   0   1;
       0   0   0   0   0   0   0   0   0   0   0   0;
       0   0   0   0   0   0   0   0   0   0   0   0;
       0   0   0   0   0   0   0   0   0   0   0   0];

  %    f     tx      ty      tz
  B = [0     0       0       0;
       0     0       0       0;
       0     0       0       0;
       0     0       0       0;
       0     0       0       0;
       1/m   0       0       0;
       0     0       0       0;
       0     0       0       0;
       0     0       0       0;
       0     1/Ix    0       0;
       0     0       1/Iy    0;
       0     0       0       1/Iz];


  pops = size(pop,1);
  f=zeros(pops,1);
  for i=1:pops
    Q = diag(10.^pop(i,1:length(X)));
    R = diag(10.^pop(i,length(X)+1:length(X)+length(U)));
    K = lqr(A, B, Q, R);
    success=0;
    %% start loop
    for j=1:length(Time)
      % Feedback Control
      Xerr = [X(1)-Xref(1); X(2)-Xref(2); X(3)-Xref(3); X(4:12)];
      U_ref = -K*Xerr + U_hover;
      % Convert ForceTorques to PWM
      PWM_ref = drone.U2PWM(U_ref);
      % filter using 1st Order Lag
      PWM = lpf.update(PWM_ref);
      % Convert Back to Force from PWM
      % This is the actual torque being produced
      U = drone.pwm2U(PWM);
      % Add Noise
      N = [0.05*rand(1,1); 0.01*rand(3,1)];
      % U = U + N;

      % Simulate in nonlinearDynamics
      dX1 = drone.nonlinearDynamics(X, U)*dt;
      dX2 = drone.nonlinearDynamics(X+dX1/2, U)*dt;
      dX3 = drone.nonlinearDynamics(X+dX2/2, U)*dt;
      dX4 = drone.nonlinearDynamics(X+dX3, U)*dt;
      X = X+(dX1+2*dX2+2*dX3+dX4)/6;
      if X(3)<0
        X(6)=-X(6);   % If it goes lower than the ground, bounce it back
      end

      % Add Noise
      N = [0.05*rand(1,3) 0.1*rand(1,3) 0.05*rand(1,3) 0.05*rand(1,3)];
      % X = X + N';

      % Estimate
      X_filtered = kf.update([X(1:3); X(7:12)]);

      % save values
      X_store(j, :) = X';
      U_store(j, :) = U';
      PWM_store(j, :) = PWM';
%       if(X(1)>0.95 && X(1)<1.05 ...
%         && X(2)>0.95 && X(2)<1.05 ...
%         && X(3)>0.95 && X(3)<1.05)
%         success=success+1;
%       end
    end

    f(i,1) = -sum(abs(Xerr));
  end
end
