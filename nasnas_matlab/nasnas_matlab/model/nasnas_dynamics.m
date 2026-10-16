function [xdot, aux] = nasnas_dynamics(x, u, wE, P, dist)
%NASNAS_DYNAMICS  Nonlinear 6-DOF model  xdot = f(x,u,w,Theta)   (Eq. 71 / 76)
%   x = [xE yE zE u v w phi theta psi p q r]'   (NED, z_E positive DOWN)
%   u = [de da dr dT]'     actual surface deflections [rad] and throttle [0..1]
%   wE   Earth-frame wind [3x1] (steady + gust)       (default zeros)
%   P    parameter struct (nasnas_params / perturb_params)
%   dist external disturbance [dX dY dZ dL dM dN]' (N, N m)   (default zeros)
if nargin < 5 || isempty(dist), dist = zeros(6,1); end
if nargin < 3 || isempty(wE),   wE   = zeros(3,1); end
vB = x(4:6);  ph = x(7); th = x(8); ps = x(9);
p = x(10); q = x(11); r = x(12);
de = u(1); da = u(2); dr = u(3); dT = min(max(u(4),0),1);

sph = sin(ph); cph = cos(ph); sth = sin(th); cth = cos(th);
R = nasnas_rot(ph, th, ps);

% --- air data, Eq.(29)-(37) ------------------------------------------------
vA = vB - R.' * wE(:);
VA = max(norm(vA), 1e-3);
alpha = atan2(vA(3), vA(1));
beta  = asin(max(min(vA(2)/VA,1),-1));
qdyn  = 0.5*P.rho*VA^2;
ph_h = p*P.b/(2*VA);  qh = q*P.c/(2*VA);  rh = r*P.b/(2*VA);   % Eq.(54)

% --- aerodynamic coefficients, Eq.(48)-(53) --------------------------------
CL = P.CL0 + P.CLa*alpha + P.CLq*qh + P.CLde*de;
CD = P.CD0 + P.CDa*alpha + P.CDa2*alpha^2 + P.CDde*abs(de);
CY = P.CYb*beta + P.CYp*ph_h + P.CYr*rh + P.CYda*da + P.CYdr*dr;
Cl = P.Clb*beta + P.Clp*ph_h + P.Clr*rh + P.Clda*da + P.Cldr*dr;
Cm = P.Cm0 + P.Cma*alpha + P.Cmq*qh + P.Cmde*de;
Cn = P.Cnb*beta + P.Cnp*ph_h + P.Cnr*rh + P.Cnda*da + P.Cndr*dr;

% --- forces and moments, Eq.(38)-(47), (25)-(28), (58) ---------------------
D = qdyn*P.S*CD;  L = qdyn*P.S*CL;  Yf = qdyn*P.S*CY;
XA = -D*cos(alpha) + L*sin(alpha);
ZA = -D*sin(alpha) - L*cos(alpha);
T  = P.Tmax*dT;
X = XA - P.m*P.g*sth        + T + dist(1);
Y = Yf + P.m*P.g*sph*cth         + dist(2);
Z = ZA + P.m*P.g*cph*cth         + dist(3);
Lm = qdyn*P.S*P.b*Cl + dist(4);
Mm = qdyn*P.S*P.c*Cm + dist(5);
Nm = qdyn*P.S*P.b*Cn + dist(6);

% --- translational dynamics, Eq.(14)-(16) ----------------------------------
udot = X/P.m - q*vB(3) + r*vB(2);
vdot = Y/P.m - r*vB(1) + p*vB(3);
wdot = Z/P.m - p*vB(2) + q*vB(1);

% --- rotational dynamics, Eq.(17)-(20) (matrix form, Sec. 2.14) ------------
I  = [P.Ix 0 -P.Ixz; 0 P.Iy 0; -P.Ixz 0 P.Iz];
om = [p; q; r];
omdot = I \ ([Lm; Mm; Nm] - cross(om, I*om));

% --- kinematics, Eq.(11)-(12) ----------------------------------------------
pos_dot = R * vB;
eul_dot = [p + sph*tan(th)*q + cph*tan(th)*r;
           cph*q - sph*r;
           (sph*q + cph*r)/cth];

xdot = [pos_dot; udot; vdot; wdot; eul_dot; omdot];

if nargout > 1
    aux = struct('VA',VA,'alpha',alpha,'beta',beta,'qdyn',qdyn,'CL',CL,'CD',CD, ...
                 'D',D,'L',L,'T',T,'forces',[X;Y;Z],'moments',[Lm;Mm;Nm]);
end
end
