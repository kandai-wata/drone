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
t_end=10;
Time = (t_start:dt:t_end)';

% Initial Condition
X = zeros(12, 1);
X_raw = X;
X_true = X;
Xref = [0 0 1]'';
U = [0 0 0 0]';
PWM = zeros(4,1);
U_hover=[m*g 0 0 0]';

% Place to be saved
X_store = zeros(length(Time), length(X));
U_store = zeros(length(Time), length(U));
PWM_store = zeros(length(Time), length(PWM));
Xraw_store = zeros(length(Time), length(X));
Xtrue_store = zeros(length(Time), length(X));

% Filter
freq=300;
lpf = LowPassFilter(PWM, dt, freq);
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
    X = zeros(12, 1);
    U = zeros(4,1);
    Q = diag(10.^[pop(i,1) pop(i,1) pop(i,2) pop(i,3) pop(i,3) pop(i,4) ...
                  pop(i,5) pop(i,5) pop(i,6) pop(i,7) pop(i,7) pop(i,8)]);
    R = diag(10.^[pop(i,9) pop(i,10) pop(i,10) pop(i,11)]);

    % Q = diag(10.^pop(i,1:length(X)));
    % R = diag(10.^pop(i,length(X)+1:length(X)+length(U)));
    % K = lqr(A, B, Q, R);
    %% start loop
    for j=1:length(Time)
      % SDRE
      psi=X(9);
      p=X(10); q=X(11); r=X(12);
      thrust = U(1);
      if thrust==0
          thrust=m*g;
      end
      a47=thrust/m*sin(psi); a48=thrust/m*cos(psi);
      a57=-thrust/m*cos(psi); a58=thrust/m*sin(psi);
      a1011=r*(Iy-Iz)/Ix;
      a1110=r*(Iz-Ix)/Iy;
      a1210=q*(Ix-Iy)/Iz/2;
      a1211=p*(Ix-Iy)/Iz/2;
      A = [0   0   0   1   0   0   0   0   0   0    0   0;
          0   0   0   0   1   0   0   0   0   0    0   0;
          0   0   0   0   0   1   0   0   0   0    0   0;
          0   0   0   0   0   0  a47 a48  0   0    0   0;
          0   0   0   0   0   0  a57 a58  0   0    0   0;
          0   0   0   0   0   0   0   0   0   0    0   0;
          0   0   0   0   0   0   0   0   0   1    0   0;
          0   0   0   0   0   0   0   0   0   0    1   0;
          0   0   0   0   0   0   0   0   0   0    0   1;
          0   0   0   0   0   0   0   0   0   0  a1011 0;
          0   0   0   0   0   0   0   0   0 a1110  0   0;
          0   0   0   0   0   0   0   0   0 a1210 a1211 0];
      K = lqr(A, B, Q, R);

      % Feedback Control
      Xerr = [X(1)-Xref(1); X(2)-Xref(2); X(3)-Xref(3); X(4:12)];
      Rot = getRotationalMatrix(X(7), X(8), X(9));
      Treq_earth = Rot(:,3)/Rot(3,3);
      Treq_body = Rot'*Treq_earth;
      U_ref = -K*Xerr + [m*g*Treq_body(3); 0; 0; 0];
      % Convert ForceTorques to PWM
      PWM_ref = drone.U2PWM(U_ref);
      % filter using 1st Order Lag
      PWM = lpf.update(PWM_ref) +2.5*wgn(4,1,0);
      % Convert Back to Force from PWM
      % This is the actual torque being produced
      U = drone.pwm2U(PWM);

      % Simulate in nonlinearDynamics
      dX1 = drone.nonlinearDynamics(X_true, U)*dt;
      dX2 = drone.nonlinearDynamics(X_true+dX1/2, U)*dt;
      dX3 = drone.nonlinearDynamics(X_true+dX2/2, U)*dt;
      dX4 = drone.nonlinearDynamics(X_true+dX3, U)*dt;
      X_true = X_true+(dX1+2*dX2+2*dX3+dX4)/6;

      % Add Noise
      N = [0.01*wgn(1,3,0) 0.01*wgn(1,3,0) 0.01*wgn(1,3,0) 0.01*wgn(1,3,0)];
      X_raw = X_true + N';

      % Estimate
      X = kf.update([X_raw(1:3); X_raw(7:12)]);

      % save values
      X_store(j, :) = X';
      U_store(j, :) = U';
      PWM_store(j, :) = PWM';
      Xraw_store(j,:) = X_raw';
      Xtrue_store(j,:) = X_true';
%       if(X(1)>0.95 && X(1)<1.05 ...
%         && X(2)>0.95 && X(2)<1.05 ...
%         && X(3)>0.95 && X(3)<1.05)
%         success=success+1;
%       end
    end
%     start=1;
%     fastforward=50;
%     record=false;
%     camera_turn=false;
%     camera_yaw=98; camera_ele=30; % camera anglec
%     bnd = [-0.1 1.2];
   %  Frames=draw_3d_animation(Time, X_store, U_store, PWM_store, dt, armlen, record, camera_turn, fastforward, camera_yaw, camera_ele, start, bnd);

    %error=[Xerr(1:3); Xerr(4:6)]'; disp(error)
    fi = 1/sum(abs([Xerr(1:3); Xerr(4:6)]));
    f(i,1) = fi;
  end
end


function R = getRotationalMatrix(phi, th, psi)
R = [cos(th)*cos(psi)       sin(phi)*sin(th)*cos(psi)-cos(phi)*sin(psi)         cos(phi)*sin(th)*cos(psi)+sin(phi)*sin(psi);
     cos(th)*sin(psi)       sin(phi)*sin(th)*sin(psi)+cos(phi)*cos(psi)         cos(phi)*sin(th)*sin(psi)-sin(phi)*cos(psi);
     -sin(th)               sin(phi)*cos(th)                                    cos(phi)*cos(th)];
end
