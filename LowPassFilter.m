classdef LowPassFilter < handle
  properties
    last;
    alpha;
  end

  methods
    function obj = LowPassFilter(init, dt, freq)
      obj.alpha = dt / (1/freq + dt);
      obj.last  = init;
    end

    function next = update(obj, curr)
        obj.last = (1-obj.alpha)*obj.last + obj.alpha*curr;
        next= obj.last;
    end
  end
end
