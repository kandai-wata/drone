classdef LowPassFilter
  properties
    last;
    dt;
    freq;
  end

  methods
    function obj = LowPassFilter(dt, freq)
      obj.dt = dt;
      obj.freq = freq;
      obj.last = 0;
    end

    function next = update(obj, curr)
      T = 1/obj.freq;
      alpha = obj.dt / (T + obj.dt);
      next = alpha*curr + (1-alpha)*obj.last;
    end
  end
end
