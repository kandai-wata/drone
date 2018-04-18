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
t_end=30;
Time = (t_start:dt:t_end)';

% Initial Condition
X = zeros(12, 1);
Xref = [0 0 1]'';
U = [0 0 0 0]';
PWM = zeros(4,1);
U_hover=[m*g 0 0 0]';

% Place to be saved
X_store = zeros(length(Time), length(X));
U_store = zeros(length(Time), length(U));
PWM_store = zeros(length(Time), length(PWM));

% Filter
freq=50;
lpf = LowPassFilter(PWM, dt, freq);
kf = KalmanFilter(dt);

% Drone
drone = Drone(m, Ix, Iy, Iz, armlen, g, dt);
A = [ 0   0   0   1   0   0   0   0   0   0   0   0;
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

Q = diag(10.^pop(1,1:length(X)));
R = diag(10.^pop(1,length(X)+1:length(X)+length(U)));
K = lqr(A, B, Q, R);
%% start loop
success=0;
for j=1:length(Time)
  % Feedback Control
  Xerr = [X(1)-Xref(1); X(2)-Xref(2); X(3)-Xref(3); X(4:12)]
  U_ref = -K*Xerr + U_hover
  % Convert ForceTorques to PWM
  PWM_ref = drone.U2PWM(U_ref)
  % filter using 1st Order Lag
  PWM = lpf.update(PWM_ref) + wgn(4,1,5/2) % add noise
  % Convert Back to Force from PWM
  % This is the actual torque being produced
  U = drone.pwm2U(PWM)

  % Simulate in nonlinearDynamics
  dX1 = drone.nonlinearDynamics(X, U)*dt;
  dX2 = drone.nonlinearDynamics(X+dX1/2, U)*dt;
  dX3 = drone.nonlinearDynamics(X+dX2/2, U)*dt;
  dX4 = drone.nonlinearDynamics(X+dX3, U)*dt;
  X = X+(dX1+2*dX2+2*dX3+dX4)/6;
%   if X(3)<0
%     X(6)=-X(6);   % If it goes lower than the ground, bounce it back
%   end

  % Add Noise
  N = [wgn(1,3,0.03) wgn(1,3,0.01) wgn(1,3,0.03) wgn(1,3,0.03)];
  X = X + N'; % add noise

  % Estimate
  X_filtered = kf.update([X(1:3); X(7:12)]);

  % save values
  X_store(j, :) = X';
  U_store(j, :) = U';
  PWM_store(j, :) = PWM';
  pause
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

%% animation
start=1;
fastforward=50;
record=true;
camera_turn=false;
camera_yaw=98; camera_ele=30; % camera anglec
% bnd = [-0.1 1.2];
bnd=[0 0];
Frames=draw_3d_animation(Time, X_store, U_store, PWM_store, dt, armlen, record, camera_turn, fastforward, camera_yaw, camera_ele, start, bnd);

%%
if record==true
    date=datetime;
    date_string = sprintf("[%i%i%i_%i%i] ", date.Year, date.Month, date.Day, date.Hour, date.Minute);
    file_name = date_string + message(find(~isspace(message)));
    vidObj = VideoWriter(char(file_name));
    open(vidObj)
    for i=1:length(Time)/fastforward
        t=fastforward*i;
        writeVideo( vidObj, Frames(t) );
    end
    close( vidObj);
end
