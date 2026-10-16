function [cmd, st] = pid_controller(out, st, dt)
%PID_CONTROLLER  Cascaded PID baseline (Sec. 4.1.5, Table 8 gains).
%   [cmd, st] = pid_controller(out, st, dt);  st = [] on the first call.
%   Outer guidance loops (altitude->pitch, airspeed->throttle, cross-track/heading->roll) and
%   inner rate loops (pitch rate->elevator, roll rate->aileron, yaw damper->rudder).
%   Outer attitude-to-rate gains (kth, kph) are [ASSUMED] (not listed in Table 8).
I = out.info;  x = I.x;  tr = I.trim;  P = I.P;
wrap = @(a) mod(a+pi, 2*pi) - pi;
if isempty(st)
    names = {'alt','q','V','ct','hd','p','yaw'};
    for n = names, st.(n{1}) = struct('I',0,'e',[],'d',0); end
    st.G = struct( ...                       % [Kp Ki Kd]  (Table 8)
        'alt',[0.025 0.005 0.010], 'q',[1.20 0.30 0.05], 'V',[0.15 0.05 0.01], ...
        'ct',[0.020 0.003 0.008], 'hd',[1.50 0.20 0.10], 'p',[0.80 0.10 0.02], 'yaw',[0.50 0.05 0.01]);
    st.kth = 2.0;  st.kph = 3.0;             % [ASSUMED] attitude -> rate gains
end
% integrator limit: +/-30 % of the actuator range (surfaces: 0.3*(2*dmax), throttle: 0.3*1)
lim_s = 0.3*2*P.dmax;  lim_t = 0.3;

th = x(8); ph = x(7); psi = wrap(x(9)); p = x(10); q = x(11); r = x(12);
% ---- longitudinal -------------------------------------------------------------
[ua, st.alt] = pid_step(st.alt, st.G.alt, I.x(3)-I.Zr, dt, lim_s);       % altitude error (NED)
thc = tr.theta + I.gam + min(max(ua,-0.5),0.5);
qc  = st.kth*(thc - th);
[uq, st.q] = pid_step(st.q, st.G.q, qc - q, dt, lim_s);
de  = tr.de - uq;                                                       % +de = nose down
[uv, st.V] = pid_step(st.V, st.G.V, I.Vr - I.VA, dt, lim_t);
dT  = min(max(tr.dT + uv, 0), 1);
% ---- lateral-directional --------------------------------------------------------
[uc, st.ct] = pid_step(st.ct, st.G.ct, -I.ec, dt, lim_s);               % cross-track -> roll
[uh, st.hd] = pid_step(st.hd, st.G.hd, wrap(I.psir - psi), dt, lim_s);  % heading -> roll
phff = atan(I.Vr*I.psidot/P.g);
phc  = min(max(phff + uc + uh, -0.7), 0.7);
pc   = st.kph*(phc - ph);
[up, st.p] = pid_step(st.p, st.G.p, pc - p, dt, lim_s);
da   = up;
[uy, st.yaw] = pid_step(st.yaw, st.G.yaw, -r, dt, lim_s);
dr   = -uy;
cmd = [min(max(de,-P.dmax),P.dmax); min(max(da,-P.dmax),P.dmax); min(max(dr,-P.dmax),P.dmax); dT];
end

function [u, s] = pid_step(s, G, e, dt, ilim)
s.I = s.I + e*dt;
if G(2) > 0, s.I = min(max(s.I, -ilim/G(2)), ilim/G(2)); end     % anti-windup clamp
if isempty(s.e), d = 0; else, d = (e - s.e)/dt; end
s.d = s.d + (d - s.d)*dt/(0.02 + dt);                            % light derivative filter
s.e = e;
u = G(1)*e + G(2)*s.I + G(3)*s.d;
end
