close all
% load pop.mat

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
X = [0 0 0 0 0 0 0 0 pi 0 0 0]';
X_raw = X;
X_true = X;
Xref = [1 1 1]';
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
freq=100;
lpf = LowPassFilter(PWM, dt, freq);
kf = KalmanFilter(X, dt);
convert = Converter;

% Drone
drone = Drone(m, Ix, Iy, Iz, armlen, g, dt);
% Q = diag(10.^pop(1,1:length(X)));
% R = diag(10.^pop(1,length(X)+1:length(X)+length(U)));
Q = diag([1 1 10 0.1 0.1 0.1 10 10 0.01 0.1 0.1 0.1]);
R = diag([0.1 0.1 0.1 1]);

% start loop
% A = [   0   0   0   1   0   0   0   0   0   0    0   0;
%     0   0   0   0   1   0   0   0   0   0    0   0;
%     0   0   0   0   0   1   0   0   0   0    0   0;
%     0   0   0   0   0   0   0   g   0   0    0   0;
%     0   0   0   0   0   0  -g   0   0   0    0   0;
%     0   0   0   0   0   0   0   0   0   0    0   0;
%     0   0   0   0   0   0   0   0   0   1    0   0;
%     0   0   0   0   0   0   0   0   0   0    1   0;
%     0   0   0   0   0   0   0   0   0   0    0   1;
%     0   0   0   0   0   0   0   0   0   0    0   0;
%     0   0   0   0   0   0   0   0   0   0    0   0;
%     0   0   0   0   0   0   0   0   0   0    0   0];
%    f     tx      ty      tz
B = [ 0     0       0       0;
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
% K = lqr(A, B, Q, R);

success=0;
for j=1:length(Time)
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
   Rot = convert.euler2RotMatrix(X(7), X(8), X(9));
   Treq_earth = Rot(:,3)/Rot(3,3);
   Treq_body = Rot'*Treq_earth;
   U_ref = -K*Xerr + [m*g*Treq_body(3); 0; 0; 0];
   % U_ref = -K*Xerr + U_hover;
  % Convert ForceTorques to PWM
  PWM_ref = drone.U2PWM(U_ref);
  % filter using 1st Order Lag
  PWM = lpf.update(PWM_ref) + 2.5*wgn(4,1,0); % add noise
  % Convert Back to Force from PWM
  % This is the actual torque being produced
  U = drone.pwm2U(PWM);
  % colNames={'Degrees','Uref','PWMref','PWM', 'U'};
  % KX = [Xerr'; zeros(1,length(Xerr)); -K]
  % sTable = array2table( [[0;180/pi*Xerr(7:9)] U_ref PWM_ref PWM U ], 'VariableNames', colNames)

  % Simulate in nonlinearDynamics
  dX1 = drone.nonlinearDynamics(X_true, U)*dt;
  dX2 = drone.nonlinearDynamics(X_true+dX1/2, U)*dt;
  dX3 = drone.nonlinearDynamics(X_true+dX2/2, U)*dt;
  dX4 = drone.nonlinearDynamics(X_true+dX3, U)*dt;
  X_true = X_true+(dX1+2*dX2+2*dX3+dX4)/6;
  if X_true(3)<0
    X_true(6)=-X_true(6);   % If it goes lower than the ground, bounce it back
  end

  % Add Noise
  N = [0.02*wgn(1,3,0) 0.01*wgn(1,3,0) 0.05*wgn(1,3,0) 0.1*wgn(1,3,0)];
  X_raw = X_true + N'; % add noise

  % Estimate
  X = kf.update(@drone.nonlinearDynamics, A, [X_raw(1:3); X_raw(7:12)], U);

  % save values
  X_store(j, :) = X';
  U_store(j, :) = U';
  PWM_store(j, :) = PWM';
  Xraw_store(j,:) = X_raw';
  Xtrue_store(j,:) = X_true';

  % Test if Kalman is doing Bad to the Simulation
  % X = X_raw;
end

%% figure
figure
subplot(2,2,1);
    % plot3(X_store(:,1), X_store(:,2), X_store(:,3) ); grid on; hold on; xlabel('x[m]'); ylabel('y[m]'); zlabel('z[m]');
    plot(Time, X_store(:,1:3)); grid on; legend('x','y','z');
subplot(2,2,2);
    plot(Time, X_store(:,7:9)); grid on; legend('phi','theta','psi');
subplot(2,2,3);
    plot(Time, U_store(:,1)); grid on;
    ylabel('thrust[N]'); ylim([0 6]);
subplot(2,2,4);
    plot(Time, U_store(:,2:4));grid on; legend('\it{tx}','ty','tz');


figure
subplot(2,2,1);
    plot(Time, Xraw_store(:,1), Time, X_store(:,1), Time, Xtrue_store(:,1));
    grid on; legend('raw', 'kalman', 'true'); ylabel('x [m]');
subplot(2,2,2);
    plot(Time, Xraw_store(:,2), Time, X_store(:,2), Time, Xtrue_store(:,2));
    grid on; legend('raw', 'kalman', 'true'); ylabel('y [m]');
subplot(2,2,3);
    plot(Time, Xraw_store(:,3), Time, X_store(:,3), Time, Xtrue_store(:,3));
    grid on; legend('raw', 'kalman', 'true'); ylabel('z [m]');

figure
subplot(2,2,1);
    plot(Time, Xraw_store(:,7), Time, X_store(:,7), Time, Xtrue_store(:,7));
    grid on; legend('raw', 'kalman', 'true'); ylabel('phi');
subplot(2,2,2);
    plot(Time, Xraw_store(:,8), Time, X_store(:,8), Time, Xtrue_store(:,8));
    grid on; legend('raw', 'kalman', 'true'); ylabel('theta');
subplot(2,2,3);
    plot(Time, Xraw_store(:,9), Time, X_store(:,9), Time, Xtrue_store(:,9));
    grid on; legend('raw', 'kalman', 'true'); ylabel('psi');
%% animation
start=0.01;
fastforward=10;
record=false;
camera_turn=false;
camera_yaw=98; camera_ele=30; % camera anglec
% bnd = [-0.1 1.2];
bnd=[0 0];
message = 'RegulatorPlusWithKalman';
currFrame = draw_3d_animation(Time, X_store, PWM_store, U_store, dt, armlen, record, camera_turn, fastforward, camera_yaw, camera_ele, start, bnd, message);
