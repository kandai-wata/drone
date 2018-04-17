% Quadrotor Hovering Simulation With AR Drone Parameters
% created on 2018/04/17
% created by Kandai Watanabe
% Lisense belongs to
% Takahashi Lab @ Keio University

calc_data = input('calc data?');
show_fig = input('show figures?');
show_anim = input('show animation?');
save_data = input('save data?');

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
t_end=5;
Time = (t_start:dt:t_end)';

% Initial Condition
X = zeros(12, 1);
Xref = [1 1 1]'';
U = [0 0 0 0]';
PWM = zeros(4,1);
U_hover=U;

% Place to be saved
X_store = zeros(length(Time), length(X));
U_store = zeros(length(Time), length(U));
PWM_store = zeros(length(Time), length(PWM));

% Controller
K = controllerLQR(m, Ix, Iy, Iz, g);
freq=50;
lpf = LowPassFilter(dt, freq);
% kf = KalmanFilter();

% Drone
drone = Drone(m, Ix, Iy, Iz, armlen, g, dt);

%% start loop
for i=1:length(Time)
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
  N = 5*rand(4,1);
  U = U + N;

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
  N = [0.05*rand(1,3) 0.1*rand(1,3) 0.0873*rand(1,3) 0.0873*rand(1,3)];
  X = X + N';

  % Estimate
  X_filtered = kalman(X, dt);

  % save values
  X_store(i, :) = X_filtered';
  U_store(i, :) = U';
  PWM_store(i, :) = PWM';
end
%% Figures
figure
subplot(2,2,1);
  plot3(X_store(:,1), X_store(:,2), X_store(:,3) ); grid on; hold on;
  xlabel('x[m]'); ylabel('y[m]'); zlabel('z[m]');
subplot(2,2,2);
  plot(Time, X_store(:,7:9)); grid on; legend('phi','theta','psi');
subplot(2,2,3);
  plot(Time, U_store(:,1)); grid on;
  ylabel('thrust[N]'); ylim([0 6]);
subplot(2,2,4);
  plot(Time, U_store(:,2:4));grid on; legend('\it{tx}','ty','tz');

%% Animation
start=1;
animate(Time,  dt, X_store, U_store, PWM_store, armlen, start);

%% Save Values
