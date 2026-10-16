function xa_next = nasnas_step(xa, cmd, wE, P, dt, dist)
%NASNAS_STEP  One RK4 step of the plant + first-order actuators (Eq. 61-63).
%   xa  = [x(12); delta(3)]  with delta = actual [de da dr] (rad)
%   cmd = [de_c da_c dr_c dT_c]  commanded (rad, rad, rad, 0..1)
if nargin < 6, dist = zeros(6,1); end
cmd = cmd(:);
cmd(1:3) = min(max(cmd(1:3), -P.dmax), P.dmax);
cmd(4)   = min(max(cmd(4), 0), 1);
k1 = aug(xa);
k2 = aug(xa + 0.5*dt*k1);
k3 = aug(xa + 0.5*dt*k2);
k4 = aug(xa + dt*k3);
xa_next = xa + dt/6*(k1 + 2*k2 + 2*k3 + k4);
xa_next(13:15) = min(max(xa_next(13:15), -P.dmax), P.dmax);
    function d = aug(z)
        del = z(13:15);
        xd  = nasnas_dynamics(z(1:12), [del; cmd(4)], wE, P, dist);
        dd  = (cmd(1:3) - del) ./ P.tau;                     % Eq.(61)
        dd  = min(max(dd, -P.drate), P.drate);               % Eq.(63)
        d   = [xd; dd];
    end
end
