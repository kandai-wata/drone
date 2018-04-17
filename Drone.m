classdef Drone
  properties
    m;
    Ix;
    Iy;
    Iz;
    armlen;
    g;
    dt;
    pwm;
  end

  methods
    function obj = Drone(m, Ix, Iy, Iz, armlen, gravity, dt)
      obj.m=m;
      obj.Ix = Ix;
      obj.Iy = Iy;
      obj.Iz = Iz;
      obj.armlen = armlen;
      obj.g = gravity;
      obj.dt = dt;
    end

    function dX = nonlinearDynamics(obj, X, U)
      % States
      u=X(4,1);     v=X(5,1);     w=X(6,1);
      phi=X(7,1);   th=X(8,1);    psi=X(9,1);
      p=X(10,1);    q=X(11,1);    r=X(12,1);

      % Force and Torques
      f = U(1,1);   tx = U(2,1);  ty = U(3,1);  tz = U(4,1);

      % obj
      m=obj.m;
      Ix = obj.Ix;
      Iy = obj.Iy;
      Iz = obj.Iz;
      g = obj.g;

      % Nonlinear Equation of Motion
      dX=[u;
          v;
          w;
          f/m*(cos(phi)*sin(th)*cos(psi)+sin(phi)*sin(psi)); % nearly equal to f/m*th (if psi=0)
          f/m*(cos(phi)*sin(th)*sin(psi)-sin(phi)*cos(psi)); % nearly equal to -f/m*phi
          f/m*(cos(phi)*cos(th))-g; % nearly equal to Uhat
          p;
          q;
          r;
          ((Iy-Iz)*q*r + tx)/Ix;
          ((Iz-Ix)*p*r + ty)/Iy;
          ((Ix-Iy)*p*q + tz)/Iz];
    end

    function PWM = U2PWM(obj, U)
      % parameters
      Cres = 3.5 * 10^(-2);
      len=obj.armlen;
      EachThrust2U = [1     1       1       1;
                      len   -len    -len    len;
                      -len  -len    len     len;
                      Cres  -Cres   Cres    -Cres];

      U2EachThrust = inv(EachThrust2U);
      each_thrusts = U2EachThrust*U;
      PWM = zeros(4,1);
      PWM(1,1) = obj.eachThrust2Pwm(each_thrusts(1), 0.0001815, 0.0087242, 0.14425);
      PWM(2,1) = obj.eachThrust2Pwm(each_thrusts(2), 0.00015618, 0.010395, 0.13894);
      PWM(3,1) = obj.eachThrust2Pwm(each_thrusts(3), 0.00014306, 0.0073295, 0.13362);
      PWM(4,1) = obj.eachThrust2Pwm(each_thrusts(4), 0.00013478, 0.0073295, 0.11698);
    end

    function pwm = eachThrust2Pwm(obj, thrust, a, b, c)
      if thrust<=(c-b^2/(4*a))
        pwm=0;
      else
        pwm = (-b + sqrt(b^2 - 4*a*(c-thrust))) / (2*a);
      end

      if pwm <=0
        pwm=0;
      end
      if pwm >100
        pwm=100;
      end
    end

    function U = pwm2U(obj, PWM)
      each_thrust = zeros(4,1);
      each_thrust(1,1) = obj.pwm2EachThrust(PWM(1), 0.0001815, 0.0087242, 0.14425);
      each_thrust(2,1) = obj.pwm2EachThrust(PWM(2), 0.00015618, 0.010395, 0.13894);
      each_thrust(3,1) = obj.pwm2EachThrust(PWM(3), 0.00014306, 0.0073295, 0.13362);
      each_thrust(4,1) = obj.pwm2EachThrust(PWM(4), 0.00013478, 0.0073295, 0.11698);
      Cres = 3.5 * 10^(-2);
      len=obj.armlen;
      EachThrust2U = [1     1       1       1;
                      len   -len    -len    len;
                      -len  -len    len     len;
                      Cres  -Cres   Cres    -Cres];
      U = EachThrust2U*each_thrust;
    end

    function thrust = pwm2EachThrust(obj, pwm, a, b, c)
      thrust = a*pwm^2 + b*pwm + c;
    end

  end

end
