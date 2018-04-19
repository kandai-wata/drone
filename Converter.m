classdef Converter
  properties
  end
  methods
    function R = euler2RotMatrix(phi, th, psi)
      R = [cos(th)*cos(psi)       sin(phi)*sin(th)*cos(psi)-cos(phi)*sin(psi)         cos(phi)*sin(th)*cos(psi)+sin(phi)*sin(psi);
          cos(th)*sin(psi)       sin(phi)*sin(th)*sin(psi)+cos(phi)*cos(psi)         cos(phi)*sin(th)*sin(psi)-sin(phi)*cos(psi);
          -sin(th)               sin(phi)*cos(th)                                    cos(phi)*cos(th)];
    end
  end
end
