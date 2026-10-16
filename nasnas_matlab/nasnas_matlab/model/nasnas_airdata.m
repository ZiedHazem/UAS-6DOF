function [VA, alpha, beta] = nasnas_airdata(x, wE)
%NASNAS_AIRDATA  Airspeed, angle of attack and sideslip from state and Earth-frame wind, Eq.(29)-(36).
R  = nasnas_rot(x(7), x(8), x(9));
vA = x(4:6) - R.' * wE(:);
VA = max(norm(vA), 1e-3);
alpha = atan2(vA(3), vA(1));
beta  = asin(max(min(vA(2)/VA, 1), -1));
end
