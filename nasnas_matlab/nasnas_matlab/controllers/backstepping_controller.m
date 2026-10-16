function [cmd, st] = backstepping_controller(out, st, dt)
%BACKSTEPPING_CONTROLLER  Lyapunov backstepping baseline, Eq.(115)-(119), Table 9 gains.
%   Deviations from the printed equations (found necessary when the law was checked against the
%   6-DOF model; please review):
%   (1) q_d = thd_dot - k2 z2 + z1   (sign of z1 term; Z is NED so a plus sign makes V1 negative)
%   (2) theta_d uses the trim angle of attack instead of the instantaneous alpha (avoids
%       alpha-dot feedback through theta_d_dot)
%   (3) the roll law (117) is scaled by Ix/(qbar S b), consistently with (115)-(116)
%   Derivatives of virtual controls are obtained by low-pass-filtered differentiation.
I = out.info;  x = I.x;  P = I.P;  tr = I.trim;
wrap = @(a) mod(a+pi, 2*pi) - pi;
k = struct('k1',0.8,'k2',1.5,'k3',2.0,'k4',0.8,'k5',2.0,'k6',1.5,'k7',2.5,'k8',1.5);
if isempty(st)
    st.prev = struct('thd',tr.theta,'qd',0,'psid',I.psir,'rd',0,'phid',0);
    st.f    = struct('thd',0,'qd',0,'psid',0,'rd',0,'phid',0);
    st.Iv = 0;  st.tauf = 0.05;
end
VA = I.VA; al = I.alpha; be = I.beta;
ph = x(7); th = x(8); psi = wrap(x(9)); p = x(10); q = x(11); r = x(12);
qbar = 0.5*P.rho*VA^2;
qh = q*P.c/(2*VA); ph_h = p*P.b/(2*VA); rh = r*P.b/(2*VA);
% ---- longitudinal ----------------------------------------------------------------
z1  = x(3) - I.Zr;                                         % altitude error (NED)
thd = tr.theta + asin(min(max((k.k1*z1 - I.Zdotr)/VA, -0.5), 0.5));
[thd_dot, st] = fdiff(st, 'thd', thd, dt, false);
z2  = th - thd;
qd  = thd_dot - k.k2*z2 + z1;
[qd_dot, st] = fdiff(st, 'qd', qd, dt, false);
de  = (1/P.Cmde)*(-P.Cm0 - P.Cma*al - P.Cmq*qh + P.Iy/(qbar*P.S*P.c)*(qd_dot - k.k3*(q - qd)));   % (115)
% ---- lateral-directional -----------------------------------------------------------
z3   = I.ec;                                               % cross-track error
psid = I.psir + asin(min(max(-k.k4*z3/VA, -0.5), 0.5));
[psid_dot, st] = fdiff(st, 'psid', psid, dt, true);
z4   = wrap(psi - psid);
rd   = psid_dot - k.k5*z4 - z3;
[rd_dot, st] = fdiff(st, 'rd', rd, dt, false);
phid = atan(VA*psid_dot/P.g);                              % (119)
[phid_dot, st] = fdiff(st, 'phid', phid, dt, false);
da = (1/P.Clda)*(-P.Clb*be - P.Clp*ph_h - P.Clr*rh + P.Ix/(qbar*P.S*P.b)* ...
        (-k.k7*(ph - phid) - k.k8*(p - phid_dot)));                                              % (117)
dr = (1/P.Cndr)*(-P.Cnb*be - P.Cnp*ph_h - P.Cnr*rh - P.Cnda*I.delta(2) + ...
        P.Iz/(qbar*P.S*P.b)*(rd_dot - k.k6*(r - rd)));                                           % (116)
% ---- throttle (PI on airspeed; not part of the backstepping design) ------------------
ev = I.Vr - VA;  st.Iv = min(max(st.Iv + ev*dt, -6), 6);
dT = min(max(tr.dT + 0.15*ev + 0.05*st.Iv, 0), 1);
cmd = [min(max(de,-P.dmax),P.dmax); min(max(da,-P.dmax),P.dmax); min(max(dr,-P.dmax),P.dmax); dT];
end

function [d, st] = fdiff(st, name, val, dt, isAngle)
dv = val - st.prev.(name);
if isAngle, dv = mod(dv+pi, 2*pi) - pi; end
raw = dv/dt;  st.prev.(name) = val;
st.f.(name) = st.f.(name) + (raw - st.f.(name))*dt/(st.tauf + dt);
d = st.f.(name);
end
